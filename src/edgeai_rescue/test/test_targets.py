"""Target file loading, midterm parking-point geometry and ordering-rule validation."""
import math
import os

import pytest

from edgeai_rescue.targets import (COOPERATIVE, NORMAL, Target, load_mission, load_targets, rescue_zones,
                                   validate_rules, zone_points)

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')
GOAL_TOLERANCE = 0.05  # Nav2 xy_goal_tolerance in config/nav2_params.yaml
MARGIN = 0.05          # MissionParams.radius_margin
ROBOT_RADIUS = 0.24


def test_midterm_file():
    targets, precedence = load_mission(os.path.join(CONFIG, 'targets_midterm.yaml'))
    by_id = {t.id: t for t in targets}
    assert by_id['S1'] == Target('S1', NORMAL, 1.0, 4.1, -1.5708, 0.5)
    assert by_id['C1'].type == COOPERATIVE and by_id['C1'].radius == 0.7
    assert precedence == []


def test_midterm_parking_points_inside_radius_with_tolerance():
    targets, _ = load_mission(os.path.join(CONFIG, 'targets_midterm.yaml'))
    for t in targets:
        for x, y, _ in zone_points(t):
            d = math.hypot(x - t.x, y - t.y)
            assert d + GOAL_TOLERANCE <= t.radius - MARGIN + 1e-9, f'{t.id}: parking point too far out'
            assert d >= 0.4 - 1e-9, f'{t.id}: parking point too close to the stand'


def test_midterm_values():
    s1 = Target('S1', NORMAL, 1.0, 4.1, -math.pi / 2, 0.5)
    c1 = Target('C1', COOPERATIVE, 4.1, 1.0, math.pi / 2, 0.7)
    (x, y, yaw), = zone_points(s1)
    assert (x, y) == pytest.approx((1.0, 3.70))
    assert yaw == pytest.approx(math.pi / 2)  # facing the marker (north)
    a, b = zone_points(c1)
    assert sorted([a[:2], b[:2]]) == [pytest.approx((3.8, 1.45)), pytest.approx((4.4, 1.45))]
    assert math.dist(a[:2], b[:2]) >= 2 * ROBOT_RADIUS + 0.1
    assert min(a[0], b[0]) - ROBOT_RADIUS > 3.45  # clear of W2's east face


def test_old_rescue_zones_wrapper_matches_old_rule():
    t = Target('T', COOPERATIVE, 3.0, 4.0, -math.pi / 2)
    (ax, ay), (bx, by) = rescue_zones(t, 0.4, 0.7)
    assert (ax, ay) == pytest.approx((3.35, 3.6))
    assert (bx, by) == pytest.approx((2.65, 3.6))


def test_load_targets_still_returns_list():
    targets = load_targets(os.path.join(CONFIG, 'targets_example.yaml'))
    assert isinstance(targets, list) and len(targets) == 7


def test_validate_rules_ok():
    assert validate_rules([['A', 'B'], ('B', 'C'), ['A', 'B']], ['A', 'B', 'C']) == [('A', 'B'), ('B', 'C')]


@pytest.mark.parametrize('rules, msg', [
    ([['A', 'X']], 'unknown'),
    ([['A', 'A']], 'itself'),
    ([['A', 'B'], ['B', 'C'], ['C', 'A']], 'cycle'),
    ([['A']], 'pair'),
])
def test_validate_rules_rejects(rules, msg):
    with pytest.raises(ValueError, match=msg):
        validate_rules(rules, ['A', 'B', 'C'])
