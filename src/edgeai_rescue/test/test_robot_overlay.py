"""robots_grid: the other robots' bodies drawn into a map-sized grid (robot_overlay node)."""
import math

import numpy as np

from edgeai_rescue.map_utils import cell_to_world
from edgeai_rescue.robot_overlay import OCCUPIED, StillTracker, robots_grid

MAP = {'origin': (-0.2, -0.3), 'resolution': 0.05}


def occupied_points(grid):
    return [cell_to_world(MAP, r, c) for r, c in zip(*np.nonzero(grid == OCCUPIED))]


def test_disc_covers_exactly_the_cells_within_the_radius():
    grid = robots_grid((100, 120), MAP['origin'], MAP['resolution'], [(2.0, 1.5)], 0.24)
    assert grid.dtype == np.int8 and set(np.unique(grid)) == {0, OCCUPIED}
    pts = occupied_points(grid)
    assert all(math.dist(p, (2.0, 1.5)) <= 0.24 for p in pts)
    # every cell centre within the radius is marked, so the disc is ~pi r^2 / res^2 cells
    assert abs(len(pts) - math.pi * 0.24 ** 2 / 0.05 ** 2) < 8
    for p in [(2.0 + 0.2, 1.5), (2.0, 1.5 - 0.2), (2.1, 1.6)]:
        r, c = int((p[1] + 0.3) // 0.05), int((p[0] + 0.2) // 0.05)
        assert grid[r, c] == OCCUPIED


def test_rows_start_at_the_map_origin():
    """Row 0 is the bottom of the map, as in nav_msgs/OccupancyGrid."""
    grid = robots_grid((100, 120), MAP['origin'], MAP['resolution'], [(0.5, 0.0)], 0.1)
    rows = np.nonzero(grid.any(axis=1))[0]
    assert rows.min() >= 3 and rows.max() <= 8   # y in [-0.1, 0.1] -> rows 4..7 from y0 = -0.3


def test_several_robots_and_clipping_at_the_map_edge():
    grid = robots_grid((40, 40), (0.0, 0.0), 0.05, [(0.0, 0.0), (1.0, 1.0), (5.0, 5.0)], 0.24)
    assert grid[0, 0] == OCCUPIED and grid[20, 20] == OCCUPIED
    assert grid[10, 10] == 0 and grid[39, 39] == 0       # (5, 5) lies off the 2 x 2 m map


def test_no_robots_gives_an_empty_grid():
    assert not robots_grid((10, 12), (0.0, 0.0), 0.05, [], 0.24).any()


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
