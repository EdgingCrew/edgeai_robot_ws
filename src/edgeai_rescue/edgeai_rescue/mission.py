"""Rescue mission logic: plan, dispatch, dwell checking, cooperative sync, ordering rules, replanning.

No ROS dependency: the rescue_coordinator node feeds it the clock, robot poses and navigation
results, executes the commands it emits and writes its log records. Unit tests drive it directly.

Flow per tick:
  * every robot works through its queue of Visits (task, zone slot, parking pose):
        IDLE -> NAVIGATING -> AT_ZONE (waiting for partner / ordering rule) -> DWELLING -> next visit
  * a task's dwell timer runs only while all its robots (1 or 2) are inside the rescue radius and
    all its predecessors are rescued; anyone leaving resets it. Robots that drift out are sent back.
  * failures, lateness, early finishes and new ordering rules request a replan. A replan is split
    into build_problem (fast, here) -> solve_problem (slow, pure, may run in a thread) -> adopt.

Times: callers pass their clock in seconds (sim time); internally everything is seconds since start().
"""
import math
import time
from dataclasses import dataclass, field

from edgeai_rescue.scheduler.cpsat_scheduler import _package, greedy_schedule, solve_schedule
from edgeai_rescue.targets import COOPERATIVE, zone_points

# Robot states
IDLE, NAVIGATING, AT_ZONE, DWELLING = 'IDLE', 'NAVIGATING', 'AT_ZONE', 'DWELLING'
# Task states
OPEN, DONE, DROPPED = 'OPEN', 'DONE', 'DROPPED'

UNREACHABLE_S = 1e4  # finite stand-in for an unreachable leg, so the solver just avoids it


@dataclass
class MissionParams:
    dwell_normal: float = 10.0       # s, continuous time inside the radius for a normal target
    dwell_coop: float = 10.0         # s, simultaneous time inside for a cooperative target
    radius_margin: float = 0.05      # m, inside = distance to target centre <= radius - margin
    normal_offset: float = 0.40      # m, parking point in front of a normal target's marker
    coop_offset: float = 0.45        # m, cooperative parking points in front of the marker ...
    coop_spacing: float = 0.6        # m, ... and this far apart, side by side
    mode: str = 'makespan'           # makespan | max_rescues
    run_time_limit: float = 420.0    # s, mission time limit
    solver_time_limit: float = 3.0   # s, first plan
    replan_time_limit: float = 1.0   # s, replans
    late_threshold: float = 5.0      # s behind the planned arrival before a replan
    switch_margin: float = 2.0       # s, a replan must beat the current plan by this much
    scheduler: str = 'cpsat'         # cpsat | greedy
    max_failures: int = 2            # failed visits before a task is dropped
    max_resends: int = 3             # goal re-sends when a robot is outside its radius after arriving
    outside_grace: float = 1.0       # s outside the radius before a robot at its zone is sent back
    min_replan_interval: float = 2.0  # s between non-urgent replans
    distance_log_period: float = 1.0  # s between 'distance' log records
    workers: int = 8                 # CP-SAT search workers
    blocking: bool = True            # plan around robots parked in corridors / doorways (travel.blocking)
    parked_radius: float = 0.24      # m, size of a parked robot for the blocking analysis (its real radius)
    block_detour: float = 1.5        # m, a leg counts as cut when the way around a parked robot is this much longer
    block_margin: float = 2.0        # s, a blocking spot counts as occupied this long after the dwell ends
    resend_shift: float = 0.03       # m, each resend aims this much deeper into the rescue disc ...
    max_goal_shift: float = 0.08     # m, ... in total at most (keeps the two cooperative robots apart)


@dataclass
class Visit:
    task: str
    slot: int
    x: float
    y: float
    yaw: float
    planned_arrival: float  # mission time
    planned_start: float
    planned_end: float


@dataclass
class RobotState:
    name: str
    state: str = IDLE
    queue: list = field(default_factory=list)  # Visits; queue[0] is the current one
    goal_seq: int = 0
    resends: int = 0
    outside_since: float | None = None
    late_flagged: bool = False
    waiting: str | None = None


@dataclass
class TaskState:
    target: object
    zones: list  # parking poses (x, y, yaw) per slot
    dwell: float
    status: str = OPEN
    dwell_start: float | None = None
    failures: int = 0
    rescued_at: float | None = None
    robots: tuple = ()


@dataclass
class Problem:
    """Snapshot handed to solve_problem; all times relative to t (mission time at build)."""
    t: float
    reason: str
    version: int
    first: bool
    robot_names: list
    robots: list      # (start node, ready time)
    tasks: list       # (zone nodes, dwell)
    task_ids: list
    slot_of_node: dict  # zone node -> (task id, slot)
    travel: list
    precedence: list  # (i, j) task indices
    release: list
    mode: str
    deadline: float   # remaining mission time
    time_limit: float
    scheduler: str
    pinned: dict      # robot name -> task id it is dwelling on
    workers: int
    blocking: list = field(default_factory=list)  # (z, u, w): leg u -> w impossible while a robot is on z
    block_margin: float = 0.0
    block_window: float = 0.0  # s a passing robot needs around a blocking spot


def solve_problem(p):
    """Pure solve step (safe to run in a worker thread). Returns result, mode used and solve time."""
    t0 = time.monotonic()
    R = len(p.robots)
    if not p.tasks or R == 0:
        res, mode = _package('EMPTY', [], p.tasks, R), p.mode
    elif p.scheduler == 'greedy':
        res, mode = greedy_schedule(p.robots, p.tasks, p.travel, p.precedence, p.release), 'makespan'
    else:
        mode = 'makespan'
        res = None
        extra = dict(time_limit_s=p.time_limit, workers=p.workers, blocking=p.blocking, block_margin=p.block_margin,
                     block_window=p.block_window)
        if p.mode == 'makespan':
            res = solve_schedule(p.robots, p.tasks, p.travel, p.precedence, p.release, mode='makespan', **extra)
        if res is None or res['makespan'] > p.deadline:
            # Not everything fits in the remaining time: rescue as many as possible before it ends
            mode = 'max_rescues'
            res = solve_schedule(p.robots, p.tasks, p.travel, p.precedence, p.release, mode='max_rescues',
                                 deadline=max(p.deadline, 0.0), **extra)
    return {'result': res, 'mode': mode, 'solve_time': time.monotonic() - t0}


class Mission:
    def __init__(self, robot_names, targets, precedence, travel, params=None, log=None):
        self.p = params or MissionParams()
        self.travel = travel  # anything with .matrix(robot_xy, zone_xy) -> seconds
        self.robots = {n: RobotState(n) for n in robot_names}
        self.tasks = {}
        for t in targets:
            coop = t.type == COOPERATIVE
            dwell = t.dwell if t.dwell is not None else (self.p.dwell_coop if coop else self.p.dwell_normal)
            zones = zone_points(t, self.p.normal_offset, self.p.coop_offset, self.p.coop_spacing)
            self.tasks[t.id] = TaskState(t, zones, dwell)
        self.precedence = list(precedence)
        self._sink = log or (lambda record: None)
        self.poses = {}
        self.outbox = []
        self.t0 = None
        self.now = 0.0
        self.version = 0
        self.replan_reason = None
        self.replan_force = False
        self.last_replan = -math.inf
        self.solving = False
        self.has_plan = False
        self.plan_end = None
        self.finished = False
        self.next_distance_log = 0.0
        self.time_limit_logged = False

    # ---------------------------------------------------------------- public API
    def precompute(self):
        """Fill the travel caches (zone distance fields, blocking analysis) before the run; slow on big maps."""
        zone_xy = [(z[0], z[1]) for t in self.tasks.values() for z in t.zones]
        groups = [k for k, t in self.tasks.items() for _ in t.zones]
        self.travel.matrix([], zone_xy)
        if self._blocking_on():
            return self.travel.blocking([], zone_xy, groups, self._clearance(), self.p.block_detour)
        return []

    def start(self, now):
        self.t0 = now
        self.now = 0.0
        self.log('start', robots=list(self.robots), precedence=[list(r) for r in self.precedence],
                 targets=[{'id': t.target.id, 'type': t.target.type, 'x': t.target.x, 'y': t.target.y,
                           'radius': t.target.radius, 'dwell': t.dwell,
                           'zones': [[round(v, 3) for v in z] for z in t.zones]}
                          for t in self.tasks.values()],
                 params=vars(self.p))
        self.request_replan('initial', force=True)

    def tick(self, now, poses):
        """Advance the state machines. poses: {robot: (x, y, yaw)} in the map frame."""
        self._set_time(now)
        self.poses.update({k: v for k, v in poses.items() if v is not None})
        if self.finished:
            return
        for r in self.robots.values():
            self._step_robot(r)
        for task in self.tasks.values():
            if task.status == OPEN:
                self._step_task(task)
        self._check_late()
        self._log_distances()
        if not self.time_limit_logged and self.now >= self.p.run_time_limit:
            self.time_limit_logged = True
            self.log('time_limit', open=[k for k, t in self.tasks.items() if t.status == OPEN])
        self._check_finished()

    def on_nav_result(self, robot, seq, success, now):
        """Result of the navigation goal `seq` sent to `robot` (stale results are ignored)."""
        self._set_time(now)
        r = self.robots[robot]
        if seq != r.goal_seq or r.state != NAVIGATING or not r.queue:
            return
        v = r.queue[0]
        if success:
            r.state = AT_ZONE
            self.log('arrived', robot=robot, task=v.task, slot=v.slot, planned=round(v.planned_arrival, 2),
                     d=self._dist(robot, self.tasks[v.task]))
        else:
            self._visit_failed(r, 'navigation failed')

    def set_precedence(self, rules):
        """New (already validated) ordering rules, e.g. from the LLM. Triggers a replan."""
        self.precedence = list(rules)
        self.log('rules', precedence=[list(r) for r in self.precedence])
        for a, b in self.precedence:
            if self.tasks[a].status == DROPPED and self.tasks[b].status == OPEN:
                self._drop(self.tasks[b], f'predecessor {a} was dropped')
        self.request_replan('rules', force=True)

    def take_commands(self):
        """Commands for the node: ('goal', robot, seq, x, y, yaw, task) and ('cancel', robot)."""
        out, self.outbox = self.outbox, []
        return out

    def request_replan(self, reason, force=False):
        if self.replan_reason is None or force:
            self.replan_reason = reason
        self.replan_force = self.replan_force or force

    def replan_due(self, now):
        """Reason for a replan that should start now, or None."""
        self._set_time(now)
        if self.finished or self.solving or self.replan_reason is None:
            return None
        if self.replan_force or self.now - self.last_replan >= self.p.min_replan_interval:
            return self.replan_reason
        return None

    def build_problem(self, now):
        """Snapshot of the remaining work as a scheduler problem, or None if nothing to plan."""
        self._set_time(now)
        reason, first = self.replan_reason or 'manual', not self.has_plan
        self.replan_reason, self.replan_force = None, False
        self.last_replan = self.now

        names, robot_xy, ready, pinned = [], [], [], {}
        for r in self.robots.values():
            head = r.queue[0] if r.queue else None
            if r.state == DWELLING and head and self.tasks[head.task].dwell_start is not None:
                task = self.tasks[head.task]
                pinned[r.name] = head.task
                names.append(r.name)
                robot_xy.append((head.x, head.y))
                ready.append(max(0.0, task.dwell_start + task.dwell - self.now))
            elif r.name in self.poses:
                names.append(r.name)
                robot_xy.append(tuple(self.poses[r.name][:2]))
                ready.append(0.0)
            else:
                self.log('no_pose', robot=r.name)
        dwelling = set(pinned.values())
        open_ids = [k for k, t in self.tasks.items() if t.status == OPEN and k not in dwelling]
        if len(names) < 2:
            coop = [k for k in open_ids if len(self.tasks[k].zones) == 2]
            if coop:
                self.log('skipped', tasks=coop, reason='fewer than 2 robots available')
            open_ids = [k for k in open_ids if k not in coop]

        zone_xy = [(z[0], z[1]) for k in open_ids for z in self.tasks[k].zones]
        zone_task = [k for k in open_ids for _ in self.tasks[k].zones]
        full = self.travel.matrix(robot_xy, zone_xy) if zone_xy else [[0.0] * len(names) for _ in names]
        cut = (self.travel.blocking(robot_xy, zone_xy, zone_task, self._clearance(), self.p.block_detour)
               if zone_xy and self._blocking_on() else [])
        R = len(names)

        # Drop tasks with a zone no robot can reach
        keep, node, zone_nodes = [], R, {}
        for k in open_ids:
            nodes = list(range(node, node + len(self.tasks[k].zones)))
            node += len(nodes)
            if all(any(math.isfinite(full[r][z]) for r in range(R)) for z in nodes):
                keep.append(k)
                zone_nodes[k] = nodes
            else:
                self._drop(self.tasks[k], 'unreachable')
        open_ids = [k for k in keep if self.tasks[k].status == OPEN]  # _drop may cascade
        old_nodes = list(range(R)) + [z for k in open_ids for z in zone_nodes[k]]
        travel = [[full[a][b] if math.isfinite(full[a][b]) else UNREACHABLE_S for b in old_nodes]
                  for a in old_nodes]

        idx = {k: i for i, k in enumerate(open_ids)}
        tasks, slot_of_node, n = [], {}, R
        for k in open_ids:
            nodes = []
            for slot in range(len(self.tasks[k].zones)):
                nodes.append(n)
                slot_of_node[n] = (k, slot)
                n += 1
            tasks.append((nodes, self.tasks[k].dwell))
        release = [0.0] * len(open_ids)
        prec = []
        for a, b in self.precedence:
            if b not in idx:
                continue
            if a in idx:
                prec.append((idx[a], idx[b]))
            elif a in dwelling:
                ta = self.tasks[a]
                release[idx[b]] = max(release[idx[b]], ta.dwell_start + ta.dwell - self.now)
        if not open_ids:
            return None
        new_index = {old: new for new, old in enumerate(old_nodes)}
        blocking = [(new_index[z], new_index[u], new_index[w]) for z, u, w in cut
                    if z in new_index and u in new_index and w in new_index]
        self.solving = True
        return Problem(t=self.now, reason=reason, version=self.version, first=first, robot_names=names,
                       robots=[(i, ready[i]) for i in range(R)], tasks=tasks, task_ids=open_ids,
                       slot_of_node=slot_of_node, travel=travel, precedence=prec, release=release,
                       mode=self.p.mode, deadline=self.p.run_time_limit - self.now,
                       time_limit=self.p.solver_time_limit if first else self.p.replan_time_limit,
                       scheduler=self.p.scheduler, pinned=pinned, workers=self.p.workers,
                       blocking=blocking, block_margin=self.p.block_margin,
                       block_window=self._clearance() / self.travel.speed if blocking else 0.0)

    def abort_solve(self, reason):
        self.solving = False
        self.log('solve_error', reason=reason)
        self.request_replan('retry', force=False)

    def adopt(self, problem, solution, now):
        """Install a solved plan if it is still current and better than the running one."""
        self._set_time(now)
        self.solving = False
        res, mode = solution['result'], solution['mode']
        if problem.version != self.version:
            self.log('replan_stale', reason=problem.reason)
            self.request_replan(problem.reason, force=True)
            return False

        t = problem.t
        routes = {}
        for i, name in enumerate(problem.robot_names):
            node, free = problem.robots[i]
            visits = []
            for p in res['routes'][i]:
                k, slot = problem.slot_of_node[p['zone']]
                x, y, yaw = self.tasks[k].zones[slot]
                arrival = t + free + problem.travel[node][p['zone']]
                visits.append(Visit(k, slot, x, y, yaw, arrival, t + p['start'], t + p['end']))
                node, free = p['zone'], p['end']
            routes[name] = visits
        planned = {problem.task_ids[p['task']] for p in res['plan']}
        skipped = [k for k in problem.task_ids if k not in planned]
        record = dict(reason=problem.reason, status=res['status'], mode=mode,
                      solve_time=round(solution['solve_time'], 3), makespan=round(t + res['makespan'], 2),
                      rescues=len(planned), skipped=skipped,
                      routes={n: [{'task': v.task, 'slot': v.slot, 'arrival': round(v.planned_arrival, 2),
                                   'start': round(v.planned_start, 2), 'end': round(v.planned_end, 2)}
                                  for v in vs] for n, vs in routes.items()})

        if self.has_plan:
            current = self._evaluate_current(problem, mode)
            better = (current is None or len(planned) > current[0]
                      or (len(planned) == current[0] and current[1] - res['makespan'] > self.p.switch_margin))
            self.log('replan', adopted=better, current_makespan=None if current is None else round(t + current[1], 2),
                     **record)
            if not better:
                return False
        else:
            self.log('plan', **record)
        if not planned and not problem.pinned:
            for k in skipped:
                self._drop(self.tasks[k], f'does not fit the remaining time ({mode})')

        for name in problem.robot_names:
            r = self.robots[name]
            new = routes[name]
            if name in problem.pinned:
                r.queue = [r.queue[0]] + new
                continue
            old_head = r.queue[0] if r.queue else None
            r.queue = new
            r.late_flagged = False
            if old_head and new and (old_head.task, old_head.slot) == (new[0].task, new[0].slot):
                continue  # same goal: keep driving / waiting
            if r.state == NAVIGATING and not new:
                self.outbox.append(('cancel', name))
            r.state, r.resends, r.waiting, r.outside_since = IDLE, 0, None, None
        self.has_plan = True
        self.plan_end = t + res['makespan']
        return True

    def replan_now(self, now):
        """Synchronous build + solve + adopt (tests, or when blocking is fine)."""
        problem = self.build_problem(now)
        if problem is None:
            return False
        return self.adopt(problem, solve_problem(problem), now)

    def summary(self):
        return {k: {'status': t.status, 'robots': list(t.robots), 'rescued_at': t.rescued_at}
                for k, t in self.tasks.items()}

    # ---------------------------------------------------------------- internals
    def log(self, event, **fields):
        self._sink({'t': round(self.now, 2), 'event': event, **fields})

    def _set_time(self, now):
        if self.t0 is None:
            raise RuntimeError('Mission.start() was not called')
        self.now = now - self.t0

    def _dist(self, robot, task):
        pose = self.poses.get(robot)
        if pose is None:
            return None
        return round(math.hypot(pose[0] - task.target.x, pose[1] - task.target.y), 3)

    def _inside(self, robot, task):
        d = self._dist(robot, task)
        return d is not None and d <= task.target.radius - self.p.radius_margin

    def _send_goal(self, r, v, shift=0.0):
        r.goal_seq += 1
        r.state = NAVIGATING
        r.outside_since = None
        r.waiting = None
        x, y = v.x, v.y
        if shift > 0:
            t = self.tasks[v.task].target
            d = math.hypot(t.x - x, t.y - y)
            if d > 1e-9:
                x, y = x + shift * (t.x - x) / d, y + shift * (t.y - y) / d
        self.outbox.append(('goal', r.name, r.goal_seq, x, y, v.yaw, v.task))
        self.log('goal_sent', robot=r.name, task=v.task, slot=v.slot, x=round(x, 3), y=round(y, 3),
                 planned_arrival=round(v.planned_arrival, 2))

    def _blocking_on(self):
        return self.p.blocking and self.p.scheduler == 'cpsat' and hasattr(self.travel, 'blocking')

    def _clearance(self):
        return self.p.parked_radius + self.travel.robot_radius

    def _step_robot(self, r):
        if not r.queue:
            r.state = IDLE
            return
        v = r.queue[0]
        task = self.tasks[v.task]
        if task.status != OPEN:
            r.queue.pop(0)
            r.state = IDLE
            return
        if r.state == IDLE:
            self._send_goal(r, v)
        elif r.state in (AT_ZONE, DWELLING):
            if self._inside(r.name, task):
                r.outside_since = None
            elif r.outside_since is None:
                r.outside_since = self.now
            elif self.now - r.outside_since >= self.p.outside_grace:
                r.resends += 1
                if r.resends > self.p.max_resends:
                    self._visit_failed(r, 'not inside the rescue radius after arriving')
                    return
                # Nav2 stopped within its goal tolerance but outside the rescue radius: resending the
                # same goal ends at the same spot, so aim deeper into the disc (towards the target)
                shift = min(r.resends * self.p.resend_shift, self.p.max_goal_shift)
                self.log('resend', robot=r.name, task=v.task, d=self._dist(r.name, task), attempt=r.resends,
                         shift=round(shift, 3))
                self._send_goal(r, v, shift)

    def _step_task(self, task):
        tid = task.target.id
        team = [r for r in self.robots.values() if r.queue and r.queue[0].task == tid]
        present = [r for r in team if r.state in (AT_ZONE, DWELLING)]
        complete = len(present) == len(task.zones)
        inside = complete and all(self._inside(r.name, task) for r in present)
        blockers = [a for a, b in self.precedence if b == tid and self.tasks[a].status != DONE]
        if task.dwell_start is not None:
            if not inside or blockers:
                out = [r.name for r in present if not self._inside(r.name, task)]
                self.log('dwell_reset', task=tid, robots=[r.name for r in team], left=out,
                         after=round(self.now - task.dwell_start, 2))
                task.dwell_start = None
                self.version += 1
                for r in team:
                    if r.state == DWELLING:
                        r.state = AT_ZONE
            elif self.now - task.dwell_start >= task.dwell - 1e-9:
                self._rescued(task, team)
            return
        if not present:
            return
        if not complete:
            for r in present:
                self._wait(r, 'partner', task=tid)
            return
        if blockers:
            for r in present:
                self._wait(r, 'precedence', task=tid, waiting_for=blockers)
            return
        if inside:
            task.dwell_start = self.now
            self.version += 1
            for r in present:
                r.state, r.waiting = DWELLING, None
            self.log('dwell_start', task=tid, robots=[r.name for r in present],
                     planned=round(present[0].queue[0].planned_start, 2),
                     d={r.name: self._dist(r.name, task) for r in present})

    def _wait(self, r, kind, **fields):
        if r.waiting != kind:
            r.waiting = kind
            self.log('wait_' + kind, robot=r.name, **fields)

    def _rescued(self, task, team):
        tid = task.target.id
        task.status, task.rescued_at = DONE, self.now
        task.robots = tuple(r.name for r in team)
        self.version += 1
        v = team[0].queue[0]
        self.log('rescued', task=tid, robots=list(task.robots), start=round(task.dwell_start, 2),
                 end=round(self.now, 2), planned_start=round(v.planned_start, 2),
                 planned_end=round(v.planned_end, 2), d={r.name: self._dist(r.name, task) for r in team})
        for r in team:
            r.queue.pop(0)
            r.state, r.resends, r.late_flagged, r.waiting, r.outside_since = IDLE, 0, False, None, None
        if self.now < v.planned_end - self.p.late_threshold:
            self.request_replan('early')
        unassigned = [k for k, t in self.tasks.items() if t.status == OPEN
                      and not any(q.task == k for r in self.robots.values() for q in r.queue)]
        if unassigned and any(not r.queue for r in team):
            self.request_replan('robot_free')

    def _visit_failed(self, r, why):
        v = r.queue[0]
        task = self.tasks[v.task]
        task.failures += 1
        self.version += 1
        self.log('failed', robot=r.name, task=v.task, slot=v.slot, reason=why, failures=task.failures)
        if task.dwell_start is not None:
            task.dwell_start = None
        r.queue, r.state, r.resends, r.waiting = [], IDLE, 0, None
        if task.failures >= self.p.max_failures:
            self._drop(task, f'failed {task.failures} times')
        self.request_replan('failed', force=True)

    def _drop(self, task, why):
        if task.status != OPEN:
            return
        task.status = DROPPED
        task.dwell_start = None
        self.version += 1
        tid = task.target.id
        self.log('dropped', task=tid, reason=why)
        for r in self.robots.values():
            head = r.queue[0] if r.queue else None
            r.queue = [q for q in r.queue if q.task != tid]
            if head and head.task == tid:
                if r.state == NAVIGATING:
                    self.outbox.append(('cancel', r.name))
                r.state, r.waiting, r.outside_since = IDLE, None, None
        for a, b in self.precedence:
            if a == tid:
                self._drop(self.tasks[b], f'predecessor {tid} was dropped')

    def _check_late(self):
        for r in self.robots.values():
            if r.state != NAVIGATING or r.late_flagged or not r.queue:
                continue
            v = r.queue[0]
            if self.now > v.planned_arrival + self.p.late_threshold:
                r.late_flagged = True
                self.log('late', robot=r.name, task=v.task, planned_arrival=round(v.planned_arrival, 2),
                         d=self._dist(r.name, self.tasks[v.task]))
                self.request_replan('late')

    def _log_distances(self):
        if self.now < self.next_distance_log:
            return
        self.next_distance_log = self.now + self.p.distance_log_period
        for r in self.robots.values():
            if r.queue:
                task = self.tasks[r.queue[0].task]
                pose = self.poses.get(r.name)
                self.log('distance', robot=r.name, task=task.target.id, d=self._dist(r.name, task), state=r.state,
                         x=None if pose is None else round(pose[0], 3), y=None if pose is None else round(pose[1], 3))

    def _check_finished(self):
        if self.has_plan and all(t.status != OPEN for t in self.tasks.values()):
            self.finished = True
            done = {k: t for k, t in self.tasks.items() if t.status == DONE}
            self.log('finished', rescued=sorted(done), dropped=sorted(k for k, t in self.tasks.items()
                                                                     if t.status == DROPPED),
                     makespan=round(max((t.rescued_at for t in done.values()), default=0.0), 2),
                     planned_makespan=None if self.plan_end is None else round(self.plan_end, 2))

    def _evaluate_current(self, problem, mode):
        """(rescues, makespan relative to problem.t) of the running plan on the new problem, or None
        if it can no longer be executed (a task is half-assigned, an ordering rule cannot hold,
        it deadlocks, misses the deadline, or leaves tasks out in makespan mode)."""
        node_of = {v: n for n, v in problem.slot_of_node.items()}
        R = len(problem.robot_names)
        seqs = []
        for name in problem.robot_names:
            q = self.robots[name].queue
            if name in problem.pinned:
                q = q[1:]
            seqs.append([(v.task, v.slot) for v in q])
        slots = {}
        for i, seq in enumerate(seqs):
            for item in seq:
                if item not in node_of:
                    return None
                slots.setdefault(item[0], []).append(i)
        idx = {k: i for i, k in enumerate(problem.task_ids)}
        covered = set()
        for k in problem.task_ids:
            n_slots = len(problem.tasks[idx[k]][0])
            have = len(slots.get(k, []))
            if have == n_slots:
                covered.add(k)
            elif have:
                return None
        if mode == 'makespan' and len(covered) != len(problem.task_ids):
            return None
        preds = {k: [problem.task_ids[i] for i, j in problem.precedence if problem.task_ids[j] == k]
                 for k in covered}
        if any(p not in covered for ps in preds.values() for p in ps):
            return None
        pos = [problem.robots[i][0] for i in range(R)]
        free = [problem.robots[i][1] for i in range(R)]
        ptr = [0] * R
        end = {}
        while len(end) < len(covered):
            progress = False
            for k in covered:
                if k in end or any(p not in end for p in preds[k]):
                    continue
                team = []
                for slot in range(len(problem.tasks[idx[k]][0])):
                    r = next((i for i in range(R) if ptr[i] < len(seqs[i]) and seqs[i][ptr[i]] == (k, slot)), None)
                    if r is None:
                        break
                    team.append((r, node_of[(k, slot)]))
                else:
                    start = max([problem.release[idx[k]]] + [end[p] for p in preds[k]]
                                + [free[r] + problem.travel[pos[r]][z] for r, z in team])
                    end[k] = start + problem.tasks[idx[k]][1]
                    for r, z in team:
                        pos[r], free[r] = z, end[k]
                        ptr[r] += 1
                    progress = True
            if not progress:
                return None  # deadlock: robots wait for each other in different orders
        makespan = max(end.values(), default=0.0)
        if makespan > problem.deadline + 1e-6 and mode == 'max_rescues':
            return None
        return len(covered), makespan
