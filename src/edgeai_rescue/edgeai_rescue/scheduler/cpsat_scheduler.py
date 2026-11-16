"""Rescue scheduler for Task 2: which robot rescues which target, in what order, and when.

No ROS dependency (only ortools), so it can be unit-tested offline and called from a node.

Problem:
  robots: list of (start_node, ready_time). ready_time > 0 when a robot is still busy at replan time.
  tasks:  list of (zones, dwell). One zone = normal target, two zones = cooperative target
          (two different robots must dwell in the two zones over the same interval).
  travel[u][v]: travel time in seconds between nodes u and v (robot starts and zone points).
  precedence: (i, j) pairs, task i must end before task j starts (Task 2B ordering rules).
  release: optional earliest start time per task, e.g. when a predecessor is still dwelling at replan time.
  blocking: optional (z, u, w) node triples: while a robot stands on node z, no robot can drive the
          leg u -> w (z is in a corridor or doorway, see travel.TravelMatrix.blocking). A robot occupies
          a zone from arrival (it drives there right after its previous dwell and waits) until its
          dwell ends + block_margin, or for good if it is its last visit; a robot occupies its start
          node until ready + block_margin, or for good if it gets no visits. A robot on leg u -> w
          passes z at departure + travel[u][z] (+- block_window); that passage must not overlap the
          occupation of z.

Modes:
  "makespan":    every task is rescued; minimize the end time of the last rescue,
                 then the sum of rescue end times as a tie-break.
  "max_rescues": tasks are optional and must end before `deadline`; maximize the number rescued,
                 then minimize makespan, then the sum of end times.

Result: dict with status, makespan, plan (one entry per rescued task) and routes (per-robot visit order).
"""
import math

from ortools.sat.python import cp_model

SCALE = 10  # model time unit = 0.1 s


def greedy_schedule(robots, tasks, travel, precedence=(), release=None):
    """Fallback and warm start: repeatedly commit the task that can finish earliest."""
    n, R = len(tasks), len(robots)
    release = release or [0.0] * n
    pos = [s for s, _ in robots]
    free = [float(t) for _, t in robots]
    end = [None] * n
    preds = [[i for i, j in precedence if j == k] for k in range(n)]
    plan = []
    while any(e is None for e in end):
        best = None
        for k in range(n):
            if end[k] is not None or any(end[p] is None for p in preds[k]):
                continue
            zones, dwell = tasks[k]
            ready = max([release[k]] + [end[p] for p in preds[k]])
            if len(zones) == 1:
                options = [(r,) for r in range(R)]
            else:
                options = [(a, b) for a in range(R) for b in range(R) if a != b]
            for team in options:
                start = max([ready] + [free[r] + travel[pos[r]][z] for r, z in zip(team, zones)])
                if best is None or (start + dwell, start) < (best[3], best[2]):
                    best = (k, team, start, start + dwell)
        if best is None:
            raise ValueError("no task can be scheduled (cycle in precedence?)")
        k, team, start, finish = best
        for r, z in zip(team, tasks[k][0]):
            pos[r], free[r] = z, finish
        end[k] = finish
        plan.append({"task": k, "robots": team, "start": start, "end": finish})
    return _package("GREEDY", plan, tasks, R)


def solve_schedule(robots, tasks, travel, precedence=(), release=None, mode="makespan", deadline=None,
                   time_limit_s=2.0, workers=8, use_greedy_hint=True, blocking=(), block_margin=0.0,
                   block_window=0.0):
    n, R = len(tasks), len(robots)
    release = release or [0.0] * n
    if mode not in ("makespan", "max_rescues"):
        raise ValueError(mode)
    if mode == "max_rescues" and deadline is None:
        raise ValueError("max_rescues needs a deadline")

    def T(sec):  # seconds -> model units, rounded up so the plan never assumes faster travel
        return int(math.ceil(sec * SCALE - 1e-9))

    greedy = greedy_schedule(robots, tasks, travel, precedence, release)
    # One visit per (task, zone slot); a cooperative task has two visits sharing one start time.
    visits = [(k, j) for k, (zones, _) in enumerate(tasks) for j in range(len(zones))]
    V = len(visits)
    if mode == "makespan":
        # The greedy plan bounds the optimal makespan. The model rounds every ready time, leg and
        # dwell up separately, so a chain can exceed T(greedy makespan) by up to one unit each;
        # without this slack the model can be infeasible with non-integer travel times and the
        # solver silently returns the greedy plan.
        H = T(greedy["makespan"]) + 2 * V + 2
        if blocking:
            # The greedy plan ignores blocking, so a blocking-respecting plan can be longer: at worst
            # the blocked legs wait for the dwells (and parked robots) in their way.
            H += sum(T(d + block_margin) for _, d in tasks) + V * T(max(max(row) for row in travel))
    else:
        H = T(deadline)

    node = [tasks[k][0][j] for k, j in visits]
    dwell = [T(d) for _, d in tasks]

    m = cp_model.CpModel()
    S = [m.NewIntVar(0, H, f"S{k}") for k in range(n)]
    if mode == "makespan":
        done = [m.NewConstant(1) for _ in range(n)]
    else:
        done = [m.NewBoolVar(f"done{k}") for k in range(n)]
        for k in range(n):
            m.Add(S[k] + dwell[k] <= H).OnlyEnforceIf(done[k])
            m.Add(S[k] == 0).OnlyEnforceIf(done[k].Not())
    for k in range(n):
        if release[k] > 0:
            m.Add(S[k] >= T(release[k])).OnlyEnforceIf(done[k])

    assign = [[None] * V for _ in range(R)]
    arcs_by_robot = []
    first_lit, last_lit, leg_lit, unused_lit = {}, {}, {}, {}  # (r, v), (r, v), (r, v, w), r -> literal
    for r, (start_node, ready) in enumerate(robots):
        arcs = []
        unused = unused_lit[r] = m.NewBoolVar(f"unused{r}")
        arcs.append((0, 0, unused))  # robot does nothing
        for v in range(V):
            k = visits[v][0]
            skip = m.NewBoolVar(f"skip{r}_{v}")
            assign[r][v] = skip.Not()
            arcs.append((v + 1, v + 1, skip))
            first = first_lit[r, v] = m.NewBoolVar(f"first{r}_{v}")
            arcs.append((0, v + 1, first))
            m.Add(S[k] >= T(ready) + T(travel[start_node][node[v]])).OnlyEnforceIf(first)
            last = last_lit[r, v] = m.NewBoolVar(f"last{r}_{v}")
            arcs.append((v + 1, 0, last))  # route ends here, no return
            for w in range(V):
                kw = visits[w][0]
                if kw == k:
                    continue  # one robot never serves both zones of the same task
                lit = leg_lit[r, v, w] = m.NewBoolVar(f"x{r}_{v}_{w}")
                arcs.append((v + 1, w + 1, lit))
                m.Add(S[kw] >= S[k] + dwell[k] + T(travel[node[v]][node[w]])).OnlyEnforceIf(lit)
        m.AddCircuit(arcs)
        arcs_by_robot.append(arcs)

    if blocking:
        _add_blocking(m, robots, visits, node, travel, dwell, S, done, H, first_lit, last_lit, leg_lit,
                      unused_lit, blocking, T(block_margin), T(block_window), T)

    for v in range(V):
        m.Add(sum(assign[r][v] for r in range(R)) == done[visits[v][0]])
    for k, (zones, _) in enumerate(tasks):
        if len(zones) == 2:
            v0, v1 = visits.index((k, 0)), visits.index((k, 1))
            for r in range(R):
                m.AddBoolOr([assign[r][v0].Not(), assign[r][v1].Not()])
    for i, j in precedence:
        m.AddImplication(done[j], done[i])
        m.Add(S[j] >= S[i] + dwell[i]).OnlyEnforceIf(done[j])

    M = m.NewIntVar(0, H, "makespan")
    for k in range(n):
        m.Add(M >= S[k] + dwell[k]).OnlyEnforceIf(done[k])
    sum_end = sum(S) + sum(dwell[k] * done[k] for k in range(n))
    w_makespan = n * (H + max(dwell, default=0)) + 1  # one unit of makespan outweighs any sum_end change
    objective = w_makespan * M + sum_end
    if mode == "max_rescues":
        w_rescue = w_makespan * (H + 1)  # one more rescue outweighs everything else
        objective += w_rescue * sum(1 - d for d in done)
    m.Minimize(objective)

    if use_greedy_hint and (mode == "makespan" or greedy["makespan"] <= deadline):
        _add_hint(m, greedy, tasks, visits, S, done if mode == "max_rescues" else None, arcs_by_robot, T)

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = time_limit_s
    solver.parameters.num_search_workers = workers
    status = solver.Solve(m)
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        if mode == "makespan":
            # Solver found nothing in time: fall back to the greedy plan (which ignores blocking)
            return dict(greedy, status="GREEDY_UNBLOCKED" if blocking else "GREEDY")
        return _package(solver.StatusName(status), [], tasks, R)

    plan = []
    for k in range(n):
        if not solver.Value(done[k]):
            continue
        team = []
        for j in range(len(tasks[k][0])):
            v = visits.index((k, j))
            team.append(next(r for r in range(R) if solver.Value(assign[r][v])))
        start = solver.Value(S[k]) / SCALE
        plan.append({"task": k, "robots": tuple(team), "start": start, "end": start + dwell[k] / SCALE})
    return _package(solver.StatusName(status), plan, tasks, R)


def _add_blocking(m, robots, visits, node, travel, dwell, S, done, H, first_lit, last_lit, leg_lit, unused_lit,
                  blocking, margin, window, T):
    """Legs through an occupied node must not overlap its occupation (see module docstring)."""
    R, V = len(robots), len(visits)
    visit_at = {node[v]: v for v in range(V)}
    start_of = {start: r for r, (start, _) in enumerate(robots)}
    occ = {}  # blocking node -> (occupied from, occupied until, enforcement literal or None)

    def occupation(z):
        if z in occ:
            return occ[z]
        if z in start_of:  # a robot still standing where it is now
            r = start_of[z]
            until = m.NewIntVar(0, H, f"occupied_start{r}")
            m.Add(until >= T(robots[r][1]) + margin)
            m.Add(until >= H).OnlyEnforceIf(unused_lit[r])  # nothing to do: it stays there
            occ[z] = (0, until, None)
            return occ[z]
        b = visit_at[z]
        k = visits[b][0]
        arrive = m.NewIntVar(0, H, f"arrive{b}")
        until = m.NewIntVar(0, H, f"occupied{b}")
        m.Add(until >= S[k] + dwell[k] + margin)
        for r, (start, ready) in enumerate(robots):
            m.Add(arrive == T(ready) + T(travel[start][node[b]])).OnlyEnforceIf(first_lit[r, b])
            m.Add(until >= H).OnlyEnforceIf(last_lit[r, b])  # stays parked there after its last rescue
            for a in range(V):
                if (r, a, b) in leg_lit:
                    ka = visits[a][0]
                    m.Add(arrive == S[ka] + dwell[ka] + T(travel[node[a]][node[b]])).OnlyEnforceIf(leg_lit[r, a, b])
        occ[z] = (arrive, until, done[k])
        return occ[z]

    for z, u, w in blocking:
        if (z not in visit_at and z not in start_of) or w not in visit_at or (u not in visit_at and u not in start_of):
            continue
        vw = visit_at[w]
        arrive, until, occupied = occupation(z)
        legs = []  # (literal, departure) of every robot that might drive u -> w
        if u in start_of:
            r = start_of[u]
            legs.append((first_lit[r, vw], T(robots[r][1])))
        else:
            vu = visit_at[u]
            ku = visits[vu][0]
            for r in range(R):
                if (r, vu, vw) in leg_lit:
                    legs.append((leg_lit[r, vu, vw], S[ku] + dwell[ku]))
        reach = T(travel[u][z])  # the traveller passes z this long after leaving u
        for lit, depart in legs:
            cond = [lit] if occupied is None else [lit, occupied]
            if isinstance(arrive, int):  # occupied from the start: pass only after it is free
                m.Add(depart + reach - window >= until).OnlyEnforceIf(cond)
                continue
            before = m.NewBoolVar("")
            m.Add(depart + reach + window <= arrive).OnlyEnforceIf(cond + [before])
            m.Add(depart + reach - window >= until).OnlyEnforceIf(cond + [before.Not()])


def _add_hint(m, greedy, tasks, visits, S, done, arcs_by_robot, T):
    by_task = {p["task"]: p for p in greedy["plan"]}
    for k in range(len(tasks)):
        m.AddHint(S[k], T(by_task[k]["start"]))
        if done is not None:
            m.AddHint(done[k], 1)
    for r, route in enumerate(greedy["routes"]):
        seq = [visits.index((p["task"], p["zone_slot"])) + 1 for p in route]
        succ = {}
        if seq:
            succ[0] = seq[0]
            for a, b in zip(seq, seq[1:]):
                succ[a] = b
            succ[seq[-1]] = 0
        else:
            succ[0] = 0
        for a, b, lit in arcs_by_robot[r]:
            if a == b and a != 0:
                m.AddHint(lit, a not in seq)
            else:
                m.AddHint(lit, succ.get(a) == b)


def _package(status, plan, tasks, R):
    plan = sorted(plan, key=lambda p: (p["start"], p["task"]))
    routes = [[] for _ in range(R)]
    for p in plan:
        for slot, r in enumerate(p["robots"]):
            routes[r].append({"task": p["task"], "zone_slot": slot, "zone": tasks[p["task"]][0][slot],
                              "start": p["start"], "end": p["end"]})
    makespan = max((p["end"] for p in plan), default=0.0)
    return {"status": status, "makespan": makespan, "plan": plan, "routes": routes}


if __name__ == "__main__":
    # Worked example from the branch and bound doc:
    # nodes 0 = R1 start, 1 = R2 start, 2 = T1, 3 = T2, 4 = zone A, 5 = zone B of cooperative TC1.
    d = {(0, 2): 8, (0, 3): 20, (0, 4): 12, (0, 5): 14, (1, 2): 18, (1, 3): 7, (1, 4): 15,
         (1, 5): 11, (2, 3): 16, (2, 4): 6, (2, 5): 9, (3, 4): 13, (3, 5): 6, (4, 5): 4, (0, 1): 10}
    travel = [[0] * 6 for _ in range(6)]
    for (a, b), v in d.items():
        travel[a][b] = travel[b][a] = v
    robots = [(0, 0.0), (1, 0.0)]
    tasks = [([2], 5), ([3], 5), ([4, 5], 10)]  # T1, T2, TC1
    for label, prec in (("no rules", ()), ("T2 before T1", [(1, 0)])):
        res = solve_schedule(robots, tasks, travel, prec)
        print(label, res["status"], res["makespan"])
        for r, route in enumerate(res["routes"]):
            print("  R%d:" % (r + 1), [(f"task{p['task']}", p["start"], p["end"]) for p in route])
    res = solve_schedule(robots, tasks, travel, mode="max_rescues", deadline=25)
    print("max_rescues by 25 s:", res["status"], [p["task"] for p in res["plan"]], res["makespan"])
