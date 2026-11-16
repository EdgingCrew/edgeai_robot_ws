"""Travel-time matrix tests on synthetic grids and on the generated midterm map."""
import math
import os

import numpy as np
import pytest

from edgeai_rescue.map_utils import load_map
from edgeai_rescue.scheduler.travel import TravelMatrix

MIDTERM_MAP = os.path.join(os.path.dirname(__file__), '..', '..', 'edgeai_robot_description', 'maps',
                           'midterm_arena.yaml')


def grid_map(grid, res=0.1):
    return {'grid': np.array(grid, dtype=np.int8), 'resolution': res, 'origin': (0.0, 0.0)}


def empty_room(n=30):
    g = np.zeros((n, n), dtype=np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    return g


def test_straight_line_distance_and_time():
    tm = TravelMatrix(grid_map(empty_room()), robot_radius=0.1, nominal_speed=0.5, goal_overhead=1.0)
    m = tm.metres((0.55, 1.05), (2.05, 1.05))
    assert m == pytest.approx(1.5, abs=0.11)
    secs = tm.matrix([(0.55, 1.05)], [(2.05, 1.05)])
    assert secs[0][0] == 0.0
    assert secs[0][1] == pytest.approx(m / 0.5 + 1.0)


def test_unreachable_sealed_box_is_inf():
    g = empty_room()
    g[10:21, 10] = g[10:21, 20] = g[10, 10:21] = g[20, 10:21] = 100  # sealed 1 m box
    tm = TravelMatrix(grid_map(g), robot_radius=0.05, nominal_speed=0.5)
    secs = tm.matrix([(0.5, 0.5)], [(1.5, 1.5), (2.5, 0.5)])
    assert math.isinf(secs[0][1]) and math.isinf(secs[1][0])
    assert math.isfinite(secs[0][2])


def test_point_inside_obstacle_snaps_to_free_cell():
    g = empty_room()
    g[14:17, 14:17] = 100
    tm = TravelMatrix(grid_map(g), robot_radius=0.1, nominal_speed=0.5)
    cell = tm.snap(1.55, 1.55)  # centre of the block
    assert cell is not None and not tm.blocked[cell]
    assert math.isfinite(tm.metres((0.5, 0.5), (1.55, 1.55)))


def test_unknown_cells_are_blocked():
    g = empty_room()
    g[:, 15] = -1  # unknown strip splits the room
    tm = TravelMatrix(grid_map(g), robot_radius=0.05, nominal_speed=0.5)
    assert math.isinf(tm.metres((0.5, 0.5), (2.5, 0.5)))


@pytest.fixture(scope='module')
def midterm():
    if not os.path.exists(MIDTERM_MAP):
        pytest.skip('midterm map not generated')
    return TravelMatrix(load_map(MIDTERM_MAP), robot_radius=0.24, nominal_speed=0.25, goal_overhead=1.0)


def test_midterm_matrix_symmetric_within_one_cell(midterm):
    pts = [(0.45, 0.5), (1.05, 0.5), (1.0, 3.70), (3.8, 1.45), (4.4, 1.45), (2.8, 4.2)]
    secs = midterm.matrix(pts[:2], pts[2:])
    one_cell = midterm.res * math.sqrt(2) / midterm.speed
    for u in range(len(pts)):
        for v in range(len(pts)):
            assert math.isfinite(secs[u][v])
            assert abs(secs[u][v] - secs[v][u]) <= one_cell + 1e-9


def test_midterm_legs_are_plausible(midterm):
    # Start -> S1 has to go around W1 through the gap east of W1b; start -> C1 around the top of W2
    s1 = midterm.metres((0.45, 0.5), (1.0, 3.70))
    c1 = midterm.metres((0.45, 0.5), (3.8, 1.45))
    assert 5.5 < s1 < 9.0
    assert 5.5 < c1 < 8.5
    assert s1 > math.dist((0.45, 0.5), (1.0, 3.70)) + 2.0  # detour, not a straight line through W1
    # The two C1 parking points are on the same side of W2, close to each other
    assert midterm.metres((3.8, 1.45), (4.4, 1.45)) < 0.8


def test_zone_rows_are_cached_robot_rows_are_not(midterm):
    midterm._cache.clear()
    midterm.matrix([(0.45, 0.5)], [(1.0, 3.70), (3.8, 1.45)])
    assert len(midterm._cache) == 2
    assert midterm.snap(0.45, 0.5) not in midterm._cache


def two_rooms(corridor_width_cells=10, detour=False):
    """Two 2 x 4 m rooms joined by a 1 m corridor around y = 1.6 m (0.1 m cells); optionally a second,
    longer 1 m corridor at y = 3.0 .. 4.0 m so the first one can be bypassed."""
    g = np.zeros((42, 72), dtype=np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[:, 21:51] = 100  # solid block between the rooms (x 2.1 .. 5.1 m)
    half = corridor_width_cells // 2
    g[16 - half:16 + half, 21:51] = 0
    if detour:
        g[30:40, 21:51] = 0
    return grid_map(g)


def test_blocking_corridor_spot_cuts_legs_between_rooms():
    tm = TravelMatrix(two_rooms(), robot_radius=0.2, nominal_speed=0.5)
    robots = [(0.5, 0.5)]
    zones = [(3.6, 1.6), (6.5, 1.6), (6.5, 2.5), (1.0, 2.8)]  # K in the corridor, E1/E2 east, W west
    groups = ['K', 'E1', 'E1', 'W']   # E1 is a cooperative target with two spots
    cut = tm.blocking(robots, zones, groups, clearance=0.4, min_detour=1.0)
    K = 1
    # robot -> east, west -> east and east -> west all go through the corridor
    assert {(z, u, w) for z, u, w in cut if z == K} == {(K, 0, 2), (K, 0, 3), (K, 4, 2), (K, 4, 3), (K, 2, 4), (K, 3, 4)}
    assert {z for z, _, _ in cut} == {K}  # spots in the open rooms block nothing
    # The two spots of one cooperative target never block each other
    assert not any(groups[z - 1] == groups[w - 1] for z, _, w in cut if z >= 1)


def test_blocking_ignores_spots_with_a_short_way_around():
    tm = TravelMatrix(two_rooms(detour=True), robot_radius=0.2, nominal_speed=0.5)
    zones = [(3.6, 1.6), (6.5, 1.6)]
    assert tm.blocking([(0.5, 0.5)], zones, ['K', 'E'], clearance=0.4, min_detour=10.0) == []
    # ... but with a stricter threshold the detour over the top counts
    assert tm.blocking([(0.5, 0.5)], zones, ['K', 'E'], clearance=0.4, min_detour=1.0) == [(1, 0, 2)]


def test_blocking_robot_position_counts_too():
    tm = TravelMatrix(two_rooms(), robot_radius=0.2, nominal_speed=0.5)
    # robot 1 stands in the corridor: robot 0 cannot get east while it is there
    cut = tm.blocking([(0.5, 0.5), (3.6, 1.6)], [(6.5, 1.6)], ['E'], clearance=0.4, min_detour=1.0)
    assert cut == [(1, 0, 2)]


def test_blocking_on_midterm_map_finds_nothing(midterm):
    # The midterm parking spots all have a way around them
    from edgeai_rescue.targets import Target, COOPERATIVE, NORMAL, zone_points
    zones = zone_points(Target('S1', NORMAL, 1.0, 4.1, -math.pi / 2, 0.5)) + \
        zone_points(Target('C1', COOPERATIVE, 4.1, 1.0, math.pi / 2, 0.7))
    assert midterm.blocking([(0.45, 0.5), (1.05, 0.5)], [z[:2] for z in zones], ['S1', 'C1', 'C1'],
                            clearance=0.52, min_detour=1.5) == []
