"""Mission logic tests with a fake world: dwell, resets, cooperative sync, ordering rules, replanning."""
import math

import pytest

from edgeai_rescue.mission import AT_ZONE, DONE, DROPPED, DWELLING, OPEN, Mission, MissionParams
from edgeai_rescue.targets import COOPERATIVE, NORMAL, Target


class EuclidTravel:
    """Straight-line travel at 1 m/s; points listed in `unreachable` cannot be reached."""

    def __init__(self, unreachable=()):
        self.unreachable = [tuple(p) for p in unreachable]

    def matrix(self, robot_xy, zone_xy):
        pts = list(robot_xy) + list(zone_xy)
        out = []
        for a in pts:
            row = []
            for b in pts:
                if a == b:
                    row.append(0.0)
                elif any(math.dist(b, u) < 1e-6 or math.dist(a, u) < 1e-6 for u in self.unreachable):
                    row.append(math.inf)
                else:
                    row.append(math.dist(a, b))
            out.append(row)
        return out


class FakeWorld:
    """Executes goals by teleporting robots after dist/speed seconds; can fail or hold goals."""

    def __init__(self, mission, poses, speed=1.0):
        self.m, self.poses, self.speed = mission, dict(poses), speed
        self.pending = {}
        self.fail_next = set()   # robots whose next goal fails
        self.always_fail = set()  # robots whose goals all fail
        self.hold = set()        # robots whose goals never finish
        self.stop_short = 0.0    # like Nav2's goal tolerance: stop this far before the goal (or stay put)
        self.goals = []
        self.now = 0.0

    def step(self, dt=0.1):
        self.now = round(self.now + dt, 6)
        for cmd in self.m.take_commands():
            if cmd[0] == 'goal':
                _, robot, seq, x, y, yaw, task = cmd
                self.goals.append((self.now, robot, task))
                arrive = self.now + math.dist(self.poses[robot][:2], (x, y)) / self.speed
                self.pending[robot] = (arrive, seq, (x, y, yaw))
            elif cmd[0] == 'cancel':
                self.pending.pop(cmd[1], None)
        for robot, (arrive, seq, pose) in list(self.pending.items()):
            if robot in self.hold or self.now < arrive:
                continue
            del self.pending[robot]
            if robot in self.fail_next or robot in self.always_fail:
                self.fail_next.discard(robot)
                self.m.on_nav_result(robot, seq, False, self.now)
            else:
                x0, y0 = self.poses[robot][:2]
                leg = math.dist((x0, y0), pose[:2])
                if leg <= self.stop_short:
                    pose = self.poses[robot]  # already "within tolerance"
                elif self.stop_short:
                    f = 1 - self.stop_short / leg
                    pose = (x0 + f * (pose[0] - x0), y0 + f * (pose[1] - y0), pose[2])
                self.poses[robot] = pose
                self.m.on_nav_result(robot, seq, True, self.now)
        self.m.tick(self.now, self.poses)
        if self.m.replan_due(self.now):
            self.m.replan_now(self.now)

    def run(self, until, dt=0.1):
        while self.now < until and not self.m.finished:
            self.step(dt)


def make(targets, robots=None, precedence=(), travel=None, **params):
    robots = robots or {'r1': (0.0, 0.0, 0.0), 'r2': (1.0, 0.0, 0.0)}
    defaults = dict(dwell_normal=5.0, dwell_coop=5.0, solver_time_limit=2.0, replan_time_limit=1.0, workers=4)
    defaults.update(params)
    log = []
    m = Mission(list(robots), targets, precedence, travel or EuclidTravel(), MissionParams(**defaults), log.append)
    m.start(0.0)
    m.tick(0.0, robots)
    m.replan_now(0.0)
    return m, FakeWorld(m, robots), log


def events(log, name):
    return [r for r in log if r['event'] == name]


NORMAL_A = Target('A', NORMAL, 3.0, 0.0, math.pi, 0.5)   # parking point (2.60, 0)
NORMAL_B = Target('B', NORMAL, -3.0, 0.0, 0.0, 0.5)
COOP = Target('C', COOPERATIVE, 0.0, 4.0, -math.pi / 2, 0.7)


def test_single_target_rescued_after_continuous_dwell():
    m, w, log = make([NORMAL_A], robots={'r1': (0.0, 0.0, 0.0)})
    w.run(30)
    assert m.finished and m.tasks['A'].status == DONE
    start, = events(log, 'dwell_start')
    rescued, = events(log, 'rescued')
    assert rescued['end'] - rescued['start'] >= 5.0 - 1e-9
    assert rescued['start'] == start['t']
    assert events(log, 'finished')[0]['rescued'] == ['A']


def test_leaving_the_zone_resets_dwell_and_robot_is_sent_back():
    m, w, log = make([NORMAL_A], robots={'r1': (0.0, 0.0, 0.0)})
    w.run(5.0)  # arrives at ~2.6 s, dwelling
    assert m.robots['r1'].state == DWELLING
    w.poses['r1'] = (1.5, 0.0, 0.0)  # pushed out of the 0.5 m radius
    w.step()
    assert len(events(log, 'dwell_reset')) == 1
    w.run(40)
    assert m.tasks['A'].status == DONE
    assert len(events(log, 'resend')) == 1
    rescued, = events(log, 'rescued')
    assert rescued['start'] > events(log, 'dwell_reset')[0]['t']


def test_cooperative_waits_for_partner_then_dwells_together():
    robots = {'r1': (0.0, 3.0, 0.0), 'r2': (0.0, -3.0, 0.0)}  # r1 is much closer
    m, w, log = make([COOP], robots=robots)
    w.run(3.0)
    assert any(r['robot'] == 'r1' for r in events(log, 'wait_partner'))
    assert not events(log, 'dwell_start')
    w.run(40)
    assert m.tasks['C'].status == DONE
    start, = events(log, 'dwell_start')
    assert sorted(start['robots']) == ['r1', 'r2']
    rescued, = events(log, 'rescued')
    assert sorted(rescued['robots']) == ['r1', 'r2']
    assert all(d <= 0.65 for d in rescued['d'].values())


def test_cooperative_dwell_resets_when_one_robot_leaves():
    m, w, log = make([COOP], robots={'r1': (0.0, 3.0, 0.0), 'r2': (0.5, 3.0, 0.0)})
    w.run(3.0)
    assert m.tasks['C'].dwell_start is not None
    w.poses['r2'] = (0.0, 2.0, 0.0)
    w.step()
    reset, = events(log, 'dwell_reset')
    assert reset['left'] == ['r2']
    assert m.robots['r1'].state == AT_ZONE  # the partner stops dwelling too
    w.run(40)
    assert m.tasks['C'].status == DONE


def test_precedence_gate_holds_dwell_until_predecessor_done():
    # r2 is right at B but B must wait for A (rescued by r1 after driving ~2.6 m + 5 s dwell)
    robots = {'r1': (0.0, 0.0, 0.0), 'r2': (-2.6, 0.0, 0.0)}
    m, w, log = make([NORMAL_A, NORMAL_B], robots=robots, precedence=[('A', 'B')])
    w.run(40)
    assert m.tasks['A'].status == DONE and m.tasks['B'].status == DONE
    a, = [r for r in events(log, 'rescued') if r['task'] == 'A']
    b_start, = [r for r in events(log, 'dwell_start') if r['task'] == 'B']
    assert b_start['t'] >= a['end']
    assert events(log, 'wait_precedence')


def test_navigation_failure_triggers_replan_and_reassignment():
    m, w, log = make([NORMAL_A], robots={'r1': (0.0, 0.0, 0.0), 'r2': (0.5, 0.0, 0.0)})
    first = m.robots['r2'].queue[0].task if m.robots['r2'].queue else None
    loser = 'r2' if first == 'A' else 'r1'
    w.fail_next.add(loser)
    w.run(40)
    assert events(log, 'failed')[0]['robot'] == loser
    assert any(r['reason'] == 'failed' for r in events(log, 'replan'))
    assert m.tasks['A'].status == DONE


def test_task_dropped_after_max_failures():
    m, w, log = make([NORMAL_A], robots={'r1': (0.0, 0.0, 0.0)}, max_failures=2)
    w.always_fail.add('r1')
    w.run(20)
    assert m.tasks['A'].status == DROPPED
    assert len(events(log, 'failed')) == 2
    assert m.finished


def test_unreachable_task_is_dropped_not_crashing():
    a_zone = (2.60, 0.0)  # NORMAL_A parking point (0.40 m in front of the stand)
    m, w, log = make([NORMAL_A, NORMAL_B], robots={'r1': (0.0, 0.0, 0.0)}, travel=EuclidTravel([a_zone]))
    assert m.tasks['A'].status == DROPPED
    assert events(log, 'dropped')[0]['reason'] == 'unreachable'
    w.run(30)
    assert m.tasks['B'].status == DONE


def test_build_problem_pins_dwelling_tasks_and_sets_release():
    robots = {'r1': (0.0, 0.0, 0.0), 'r2': (5.0, 5.0, 0.0)}
    m, w, log = make([NORMAL_A, NORMAL_B], robots=robots, precedence=[('A', 'B')])
    w.run(4.0)  # r1 dwelling at A since ~2.7 s
    dweller = next(n for n, r in m.robots.items() if r.state == DWELLING)
    p = m.build_problem(w.now)
    assert p.pinned == {dweller: 'A'}
    assert p.task_ids == ['B']
    i = p.robot_names.index(dweller)
    remaining = m.tasks['A'].dwell_start + 5.0 - p.t
    assert p.robots[i][1] == pytest.approx(remaining)
    assert p.release == [pytest.approx(remaining)]
    m.solving = False


def test_time_limit_switches_to_max_rescues():
    targets = [NORMAL_A, NORMAL_B]
    m, w, log = make(targets, robots={'r1': (0.0, 0.0, 0.0)}, run_time_limit=12.0)
    plan, = events(log, 'plan')
    assert plan['mode'] == 'max_rescues'
    assert plan['rescues'] == 1  # 2.6 + 5 fits, then 5.2 more + 5 does not
    w.run(30)
    assert sum(t.status == DONE for t in m.tasks.values()) == 1


def test_rules_update_replans_and_reorders():
    robots = {'r1': (0.0, 0.0, 0.0)}
    m, w, log = make([NORMAL_A, NORMAL_B], robots=robots)
    m.set_precedence([('B', 'A')])
    w.step()
    assert [v.task for v in m.robots['r1'].queue] == ['B', 'A']
    w.run(60)
    a, b = (next(r for r in events(log, 'rescued') if r['task'] == k) for k in 'AB')
    assert b['end'] <= a['start']


def test_greedy_scheduler_option():
    m, w, log = make([NORMAL_A, COOP], scheduler='greedy')
    assert events(log, 'plan')[0]['status'] == 'GREEDY'
    w.run(60)
    assert all(t.status == DONE for t in m.tasks.values())


def test_stale_solution_is_not_adopted():
    m, w, log = make([NORMAL_A, NORMAL_B], robots={'r1': (0.0, 0.0, 0.0), 'r2': (0.5, 0.0, 0.0)})
    m.request_replan('manual', force=True)
    p = m.build_problem(0.0)
    from edgeai_rescue.mission import solve_problem
    sol = solve_problem(p)
    m.version += 1  # something happened while solving
    assert m.adopt(p, sol, 0.0) is False
    assert m.replan_reason == 'manual'
    assert events(log, 'replan_stale')


def test_resend_aims_deeper_when_nav2_stops_short_outside_the_radius():
    # Parking point A is 0.40 m from the centre; stopping 0.07 m short is 0.47 m > 0.45 (radius - margin).
    # Resending the same goal ends at the same spot; the shifted resends get the robot inside.
    m, w, log = make([NORMAL_A], robots={'r1': (0.0, 0.0, 0.0)})
    w.stop_short = 0.07
    w.run(40)
    assert m.tasks['A'].status == DONE
    shifts = [r['shift'] for r in events(log, 'resend')]
    assert shifts and shifts == sorted(shifts) and max(shifts) <= m.p.max_goal_shift
    assert events(log, 'rescued')[0]['d']['r1'] <= 0.45


def test_resend_without_shift_cannot_recover_from_stopping_short():
    m, w, log = make([NORMAL_A], robots={'r1': (0.0, 0.0, 0.0)}, resend_shift=0.0, max_failures=1)
    w.stop_short = 0.07
    w.run(40)
    assert m.tasks['A'].status == DROPPED
