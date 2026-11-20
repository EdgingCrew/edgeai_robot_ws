"""robots_grid: the other robots' bodies drawn into a map-sized grid (robot_overlay node)."""
import math

import numpy as np

from edgeai_rescue.map_utils import cell_to_world
from edgeai_rescue.robot_overlay import OCCUPIED, StillTracker, robots_grid

MAP = {'origin': (-0.2, -0.3), 'resolution': 0.05}


def occupied_points(grid, m=MAP):
    return [cell_to_world(m, r, c) for r, c in zip(*np.nonzero(grid == OCCUPIED))]


BOX = (-0.2175, 0.2175, -0.19, 0.19)   # node default: 0.375 x 0.32 m body + 0.03 m margin


def inside_box(p, pose, box=BOX, tol=1e-9):
    dx, dy = p[0] - pose[0], p[1] - pose[1]
    u = math.cos(pose[2]) * dx + math.sin(pose[2]) * dy
    v = -math.sin(pose[2]) * dx + math.cos(pose[2]) * dy
    return box[0] - tol <= u <= box[1] + tol and box[2] - tol <= v <= box[3] + tol


def test_body_covers_exactly_the_cells_inside_the_rectangle():
    pose = (2.0, 1.5, 0.0)
    grid = robots_grid((100, 120), MAP['origin'], MAP['resolution'], [pose], BOX)
    assert grid.dtype == np.int8 and set(np.unique(grid)) == {0, OCCUPIED}
    pts = occupied_points(grid)
    assert all(inside_box(p, pose) for p in pts)
    assert abs(len(pts) - 0.435 * 0.38 / 0.05 ** 2) < 10      # ~ area / cell area


LONG = (-0.3, 0.3, -0.1, 0.1)


def test_body_turns_with_the_robot():
    def extent(yaw):
        pose = (2.0, 1.5, yaw)
        pts = occupied_points(robots_grid((100, 120), MAP['origin'], MAP['resolution'], [pose], LONG))
        assert pts and all(inside_box(p, pose, LONG) for p in pts)
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        return max(xs) - min(xs), max(ys) - min(ys)
    ex, ey = extent(0.0)
    assert ex > 0.5 and ey < 0.2                              # long side along the heading
    ex, ey = extent(math.pi / 2)
    assert ey > 0.5 and ex < 0.2
    ex, ey = extent(math.pi / 4)
    assert 0.3 < ex < 0.5 and 0.3 < ey < 0.5


def test_rows_start_at_the_map_origin():
    """Row 0 is the bottom of the map, as in nav_msgs/OccupancyGrid."""
    grid = robots_grid((100, 120), MAP['origin'], MAP['resolution'], [(0.5, 0.0, 0.0)], (-0.1, 0.1, -0.1, 0.1))
    rows = np.nonzero(grid.any(axis=1))[0]
    assert rows.min() >= 3 and rows.max() <= 8   # y in [-0.1, 0.1] -> rows 4..7 from y0 = -0.3


def test_several_robots_and_clipping_at_the_map_edge():
    poses = [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (5.0, 5.0, 0.0)]
    grid = robots_grid((40, 40), (0.0, 0.0), 0.05, poses, BOX)
    assert grid[0, 0] == OCCUPIED and grid[20, 20] == OCCUPIED
    assert grid[10, 10] == 0 and grid[39, 39] == 0       # (5, 5) lies off the 2 x 2 m map


def test_no_robots_gives_an_empty_grid():
    assert not robots_grid((10, 12), (0.0, 0.0), 0.05, [], BOX).any()


def test_still_tracker_needs_still_time_without_moving():
    tr = StillTracker(distance=0.05, time=1.0)
    assert not tr.still('r', 0.0)                 # no pose yet
    tr.update('r', 1.0, 1.0, 0.0)
    assert not tr.still('r', 0.5) and tr.still('r', 1.0)
    tr.update('r', 1.03, 1.0, 1.2)                # jitter within 5 cm: still standing
    assert tr.still('r', 1.2)
    tr.update('r', 1.2, 1.0, 1.4)                 # drives off: moving at once
    assert not tr.still('r', 1.4) and not tr.still('r', 2.3)
    assert tr.still('r', 2.4)


def test_cells_around_the_receiving_robot_stay_free():
    """Two robots 0.4 m apart: the other robot's body is cut back to clear_radius around this one,
    so this robot is not inside an obstacle (inscribed radius 0.24 / 0.28 m) and can drive away."""
    me, other = (1.0, 1.0), (1.4, 1.0, 0.0)
    full = robots_grid((60, 60), (0.0, 0.0), 0.05, [other], BOX)
    cut = robots_grid((60, 60), (0.0, 0.0), 0.05, [other], BOX, clear=(*me, 0.35))
    origin0 = {'origin': (0.0, 0.0), 'resolution': 0.05}
    pts_full, pts_cut = occupied_points(full, origin0), occupied_points(cut, origin0)
    assert min(math.dist(p, me) for p in pts_full) < 0.28                   # without the cut: inside
    assert min(math.dist(p, me) for p in pts_cut) > 0.35 - 1e-9             # with it: free around me
    assert 0 < len(pts_cut) < len(pts_full)                                 # the far side remains
    assert max(p[0] for p in pts_cut) == max(p[0] for p in pts_full)
