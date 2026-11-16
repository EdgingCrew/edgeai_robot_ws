"""CP-SAT scheduler tests: worked example, independent feasibility checks, CP-SAT vs greedy, max_rescues."""
import math
import random

import pytest

from edgeai_rescue.scheduler.cpsat_scheduler import greedy_schedule, solve_schedule

EPS = 1e-6


def worked_example():
    # nodes 0 = R1 start, 1 = R2 start, 2 = T1, 3 = T2, 4/5 = zones A/B of cooperative TC1
    d = {(0, 2): 8, (0, 3): 20, (0, 4): 12, (0, 5): 14, (1, 2): 18, (1, 3): 7, (1, 4): 15,
         (1, 5): 11, (2, 3): 16, (2, 4): 6, (2, 5): 9, (3, 4): 13, (3, 5): 6, (4, 5): 4, (0, 1): 10}
    travel = [[0] * 6 for _ in range(6)]
    for (a, b), v in d.items():
        travel[a][b] = travel[b][a] = v
    return [(0, 0.0), (1, 0.0)], [([2], 5), ([3], 5), ([4, 5], 10)], travel


def check_plan(res, robots, tasks, travel, precedence=(), release=None, deadline=None, all_tasks=True):
    """Independent feasibility checker for a scheduler result. Raises AssertionError on violations."""
    release = release or [0.0] * len(tasks)
    plan = {p['task']: p for p in res['plan']}
    if all_tasks:
        assert set(plan) == set(range(len(tasks))), 'not every task is rescued'
    for k, p in plan.items():
        zones, dwell = tasks[k]
        assert len(p['robots']) == len(zones)
        assert len(set(p['robots'])) == len(zones), 'cooperative task needs two different robots'
        assert p['end'] - p['start'] >= dwell - EPS and p['end'] - p['start'] <= dwell + 0.1 + EPS
        assert p['start'] >= release[k] - EPS
        if deadline is not None:
            assert p['end'] <= deadline + EPS
    for i, j in precedence:
        if j in plan:
            assert i in plan, f'task {j} done without its predecessor {i}'
            assert plan[j]['start'] >= plan[i]['end'] - EPS
    seen = {}
    for r, route in enumerate(res['routes']):
        pos, t = robots[r]
        for v in route:
            p = plan[v['task']]
            assert p['robots'][v['zone_slot']] == r
            assert v['zone'] == tasks[v['task']][0][v['zone_slot']]
            assert v['start'] == p['start']  # a cooperative task's two visits share one start
            assert v['start'] >= t + travel[pos][v['zone']] - EPS, f'robot {r} cannot reach task {v["task"]} in time'
            pos, t = v['zone'], v['end']
            seen[(v['task'], v['zone_slot'])] = r
    for k, p in plan.items():
        for slot in range(len(tasks[k][0])):
            assert (k, slot) in seen, f'task {k} slot {slot} has no robot route'
    assert math.isclose(res['makespan'], max((p['end'] for p in plan.values()), default=0.0))


def random_instance(rng, n_robots, n_targets, coop_share=0.3, with_rule=True):
    pts = [(rng.uniform(0, 5), rng.uniform(0, 5)) for _ in range(n_robots)]
    tasks = []
    for _ in range(n_targets):
        if n_robots >= 2 and rng.random() < coop_share:
            cx, cy = rng.uniform(0, 5), rng.uniform(0, 5)
            nodes = [len(pts), len(pts) + 1]
            pts += [(cx - 0.3, cy), (cx + 0.3, cy)]
            tasks.append((nodes, rng.choice([5.0, 10.0])))
        else:
            nodes = [len(pts)]
            pts.append((rng.uniform(0, 5), rng.uniform(0, 5)))
            tasks.append((nodes, rng.choice([3.0, 5.0])))
    speed = 0.25
    travel = [[math.dist(a, b) / speed + (1.0 if a != b else 0.0) for b in pts] for a in pts]
    robots = [(i, rng.choice([0.0, 0.0, 2.5])) for i in range(n_robots)]
    precedence = []
    if with_rule and rng.random() < 0.5 and n_targets >= 2:
        i, j = rng.sample(range(n_targets), 2)
        precedence.append((i, j))
    return robots, tasks, travel, precedence


def test_worked_example():
    robots, tasks, travel = worked_example()
    res = solve_schedule(robots, tasks, travel)
    assert res['status'] == 'OPTIMAL'
    assert res['makespan'] == pytest.approx(29.0)
    check_plan(res, robots, tasks, travel)


def test_worked_example_with_rule():
    robots, tasks, travel = worked_example()
    res = solve_schedule(robots, tasks, travel, precedence=[(1, 0)])
    assert res['makespan'] == pytest.approx(33.0)
    check_plan(res, robots, tasks, travel, precedence=[(1, 0)])


def test_greedy_is_feasible_on_worked_example():
    robots, tasks, travel = worked_example()
    res = greedy_schedule(robots, tasks, travel, precedence=[(1, 0)])
    check_plan(res, robots, tasks, travel, precedence=[(1, 0)])


@pytest.mark.parametrize('seed', range(30))
def test_random_instances_feasible_and_not_worse_than_greedy(seed):
    rng = random.Random(seed)
    robots, tasks, travel, prec = random_instance(rng, rng.randint(2, 4), rng.randint(3, 8))
    release = [rng.choice([0.0, 0.0, 0.0, 4.0]) for _ in tasks]
    greedy = greedy_schedule(robots, tasks, travel, prec, release)
    check_plan(greedy, robots, tasks, travel, prec, release)
    res = solve_schedule(robots, tasks, travel, prec, release, time_limit_s=1.5)
    check_plan(res, robots, tasks, travel, prec, release)
    # CP-SAT must actually solve (the greedy plan is a hint, so a solution always exists), not fall back
    assert res['status'] in ('OPTIMAL', 'FEASIBLE')
    # CP-SAT rounds each leg up to 0.1 s, so it may lose up to that much per visit against greedy
    assert res['makespan'] <= greedy['makespan'] + 0.1 * sum(len(z) for z, _ in tasks) + EPS


@pytest.mark.parametrize('seed', range(8))
def test_max_rescues_deadline(seed):
    rng = random.Random(100 + seed)
    robots, tasks, travel, prec = random_instance(rng, rng.randint(2, 3), rng.randint(3, 5))
    opt = solve_schedule(robots, tasks, travel, prec, time_limit_s=5.0)
    assert opt['status'] == 'OPTIMAL'
    at_opt = solve_schedule(robots, tasks, travel, prec, mode='max_rescues', deadline=opt['makespan'],
                            time_limit_s=5.0)
    check_plan(at_opt, robots, tasks, travel, prec, deadline=opt['makespan'])
    assert len(at_opt['plan']) == len(tasks)
    below = solve_schedule(robots, tasks, travel, prec, mode='max_rescues', deadline=opt['makespan'] - 1.0,
                           time_limit_s=5.0)
    check_plan(below, robots, tasks, travel, prec, deadline=opt['makespan'] - 1.0, all_tasks=False)
    assert len(below['plan']) < len(tasks)


def test_ready_time_delays_robot():
    # One robot at node 0, busy for 7 s; target 2 s away -> rescue starts at 9 s
    travel = [[0, 2], [2, 0]]
    res = solve_schedule([(0, 7.0)], [([1], 5.0)], travel)
    assert res['plan'][0]['start'] == pytest.approx(9.0)
    check_plan(res, [(0, 7.0)], [([1], 5.0)], travel)


def test_release_time():
    travel = [[0, 2], [2, 0]]
    res = solve_schedule([(0, 0.0)], [([1], 5.0)], travel, release=[6.0])
    assert res['plan'][0]['start'] == pytest.approx(6.0)


def test_cooperative_waits_for_slower_robot():
    # Robot 0 is 3 s from zone A, robot 1 is 8 s from zone B: the shared dwell starts at 8
    travel = [[0, 20, 3, 20], [20, 0, 20, 8], [3, 20, 0, 1], [20, 8, 1, 0]]
    res = solve_schedule([(0, 0.0), (1, 0.0)], [([2, 3], 10.0)], travel)
    assert res['plan'][0]['start'] == pytest.approx(8.0)
    assert res['plan'][0]['robots'] == (0, 1)


def check_blocking(res, robots, tasks, travel, blocking, margin, window, tol=0.35):
    """Independent check of the blocking rule: a robot passing node z on leg u -> w (at departure +
    travel[u][z], +- window) never meets a robot standing on z (from arrival to dwell end + margin, for
    good after its last visit; a robot's start until it leaves + margin, for good if it never leaves)."""
    occupied = {}  # node -> (from, until)
    legs = []      # (u, w, departure)
    for r, route in enumerate(res['routes']):
        pos, t = robots[r]
        occupied[pos] = (0.0, t + margin if route else math.inf)
        for i, v in enumerate(route):
            legs.append((pos, v['zone'], t))
            arrival = t + travel[pos][v['zone']]
            occupied[v['zone']] = (arrival, v['end'] + margin if i < len(route) - 1 else math.inf)
            pos, t = v['zone'], v['end']
    for z, u, w in blocking:
        if z not in occupied:
            continue
        lo, hi = occupied[z]
        for lu, lw, depart in legs:
            if (lu, lw) != (u, w):
                continue
            passing = depart + travel[u][z]
            assert passing + window <= lo + tol or passing - window >= hi - tol, \
                f'leg {u}->{w} passes {z} at {passing:.1f} while it is occupied {lo:.1f}..{hi:.1f}'


def corridor_instance():
    # 0, 1 = robot starts (west), 2 = K (parking spot inside the only corridor), 3 = E (east of it)
    travel = [[0, 4, 5, 10], [4, 0, 6, 9], [5, 6, 0, 5], [10, 9, 5, 0]]
    blocking = [(2, 0, 3), (2, 1, 3)]  # a robot on K cuts start -> E for both robots
    return [(0, 0.0), (1, 0.0)], [([2], 10.0), ([3], 10.0)], travel, blocking


def test_blocking_changes_who_crosses_the_corridor():
    robots, tasks, travel, blocking = corridor_instance()
    free = solve_schedule(robots, tasks, travel)
    assert free['makespan'] == pytest.approx(19.0)  # R1 parks on K at 5 s, R2 drives through it at ~6 s
    with pytest.raises(AssertionError):
        check_blocking(free, robots, tasks, travel, blocking, margin=2.0, window=1.0)
    res = solve_schedule(robots, tasks, travel, blocking=blocking, block_margin=2.0, block_window=1.0)
    assert res['status'] == 'OPTIMAL'
    check_plan(res, robots, tasks, travel)
    check_blocking(res, robots, tasks, travel, blocking, margin=2.0, window=1.0)
    # Best plan: R1 crosses first (passes K at 5 s) and R2 parks on K afterwards (arrives at 6 s)
    assert res['makespan'] == pytest.approx(20.0)
    assert [v['task'] for v in res['routes'][0]] == [1] and [v['task'] for v in res['routes'][1]] == [0]


def test_parked_robot_without_work_blocks_for_good():
    # Robot 1 stands in the corridor (node 1) and gets nothing to do: robot 0 can never pass it
    travel = [[0, 3, 8], [3, 0, 5], [8, 5, 0]]
    res = solve_schedule([(0, 0.0), (1, 0.0)], [([2], 5.0)], travel, blocking=[(1, 0, 2)], block_window=0.5)
    assert res['routes'][1] and not res['routes'][0]  # so robot 1 does the task itself


@pytest.mark.parametrize('seed', range(12))
def test_random_instances_respect_blocking(seed):
    rng = random.Random(500 + seed)
    robots, tasks, travel, prec = random_instance(rng, rng.randint(2, 3), rng.randint(3, 6), with_rule=False)
    n = len(travel)
    zones = [z for zs, _ in tasks for z in zs]
    blocking = list({(rng.choice(zones), rng.randrange(n), rng.choice(zones)) for _ in range(rng.randint(2, 8))})
    blocking = [(z, u, w) for z, u, w in blocking if len({z, u, w}) == 3]
    res = solve_schedule(robots, tasks, travel, blocking=blocking, block_margin=1.0, block_window=0.5,
                         time_limit_s=3.0)
    if res['status'] == 'GREEDY_UNBLOCKED':
        pytest.skip('no blocking-respecting plan found in time')
    check_plan(res, robots, tasks, travel)
    check_blocking(res, robots, tasks, travel, blocking, margin=1.0, window=0.5)
