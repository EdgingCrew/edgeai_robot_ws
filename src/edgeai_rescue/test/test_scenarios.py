"""Every scenario in edgeai_robot_description/scenarios: generated files up to date, geometry that
Nav2 can actually drive, and a complete first plan.

The geometry checks use the coordinator's parameters (config/rescue_coordinator.yaml), so a change
there (e.g. a larger robot_radius or other parking offsets) is checked against all scenarios.
"""
import dataclasses
import glob
import importlib.util
import itertools
import math
import os

import numpy as np
import pytest
import yaml

from edgeai_rescue.map_utils import load_map, read_pgm, world_to_cell
from edgeai_rescue.mission import Mission, MissionParams, solve_problem
from edgeai_rescue.scheduler.travel import TravelMatrix
from edgeai_rescue.targets import load_mission, zone_points

SRC = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
DESCRIPTION = os.path.join(SRC, 'edgeai_robot_description')
CONFIG = os.path.join(SRC, 'edgeai_rescue', 'config')
SCENARIOS = sorted(glob.glob(os.path.join(DESCRIPTION, 'scenarios', '*.yaml')))
GOAL_TOLERANCE = 0.05   # Nav2 xy_goal_tolerance (config/nav2_params.yaml)
ROBOT_FOOTPRINT = 0.24  # Nav2 local costmap robot_radius: the robot's real size

with open(os.path.join(CONFIG, 'rescue_coordinator.yaml')) as f:
    COORDINATOR = yaml.safe_load(f)['rescue_coordinator']['ros__parameters']
PARAMS = MissionParams(**{k: v for k, v in COORDINATOR.items()
                          if k in {f.name for f in dataclasses.fields(MissionParams)}})


def _make_world():
    spec = importlib.util.spec_from_file_location('make_world', os.path.join(DESCRIPTION, 'scripts', 'make_world.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


make_world = _make_world()


class Scenario:
    def __init__(self, path):
        self.name = os.path.splitext(os.path.basename(path))[0]
        self.path = path
        with open(path) as f:
            self.data = yaml.safe_load(f)
        self.robots = self.data['robots']
        self.world = os.path.join(DESCRIPTION, 'worlds', self.data['world'])
        self.map = os.path.join(DESCRIPTION, 'maps', self.data['map'])
        self.targets_file = os.path.join(CONFIG, self.data['targets'])
        self.targets, self.precedence = load_mission(self.targets_file)
        with open(self.targets_file) as f:
            self.markers = {e['id']: e.get('marker') for e in yaml.safe_load(f)['targets']}
        self.grid = load_map(self.map)
        self.travel = TravelMatrix(self.grid, COORDINATOR['robot_radius'], COORDINATOR['nominal_speed'],
                                   COORDINATOR['goal_overhead'])

    def zones(self):
        return {t.id: zone_points(t, PARAMS.normal_offset, PARAMS.coop_offset, PARAMS.coop_spacing)
                for t in self.targets}

    def free(self, x, y):
        """The robot centre may stand at (x, y) without snapping: inside the map and outside the inflation."""
        r, c = world_to_cell(self.grid, x, y)
        return 0 <= r < self.travel.h and 0 <= c < self.travel.w and not self.travel.blocked[r, c]


@pytest.fixture(scope='module', params=SCENARIOS, ids=lambda p: os.path.splitext(os.path.basename(p))[0])
def scenario(request):
    return Scenario(request.param)


def test_scenarios_found():
    names = {os.path.splitext(os.path.basename(p))[0] for p in SCENARIOS}
    assert {'midterm', 'midterm_plus', 'rooms', 'warehouse'} <= names


def test_generated_world_and_map_are_up_to_date(scenario):
    _, text, _ = make_world.world_sdf(scenario.path)
    with open(scenario.world) as f:
        assert f.read() == text, f'run scripts/make_world.py {scenario.name}'
    img, origin = make_world.world_map(scenario.world)
    stem = os.path.splitext(scenario.map)[0]
    assert np.array_equal(read_pgm(stem + '.pgm'), img), f'run scripts/make_world.py {scenario.name}'
    with open(scenario.map) as f:
        assert yaml.safe_load(f)['origin'][:2] == pytest.approx(list(origin))


def test_markers_unique_and_models_exist(scenario):
    markers = list(scenario.markers.values())
    assert None not in markers
    assert len(set(markers)) == len(markers), 'two stands with the same ArUco id'
    for m in markers:
        assert os.path.isdir(os.path.join(DESCRIPTION, 'models', f'aruco_stand_{m}'))


def test_robot_starts_free_and_apart(scenario):
    names = [r['name'] for r in scenario.robots]
    assert len(set(names)) == len(names)
    for r in scenario.robots:
        assert scenario.free(r['x'], r['y']), f"{r['name']} spawns inside the inflated obstacles"
    for a, b in itertools.combinations(scenario.robots, 2):
        assert math.dist((a['x'], a['y']), (b['x'], b['y'])) >= 2 * ROBOT_FOOTPRINT + 0.1


def test_parking_points_valid(scenario):
    zones = scenario.zones()
    points = []
    for t in scenario.targets:
        for x, y, _ in zones[t.id]:
            d = math.hypot(x - t.x, y - t.y)
            assert d + GOAL_TOLERANCE <= t.radius - PARAMS.radius_margin + 1e-9, f'{t.id}: parking point too far out'
            assert scenario.free(x, y), f'{t.id}: parking point ({x:.2f}, {y:.2f}) inside the inflated obstacles'
            points.append((t.id, x, y))
    # Robots parked at different spots (incl. the two of a cooperative target) must not overlap
    for (ta, xa, ya), (tb, xb, yb) in itertools.combinations(points, 2):
        assert math.dist((xa, ya), (xb, yb)) >= 2 * ROBOT_FOOTPRINT + 0.05, f'{ta} and {tb} parking spots overlap'
    for r in scenario.robots:
        for t, x, y in points:
            assert math.dist((r['x'], r['y']), (x, y)) >= 2 * ROBOT_FOOTPRINT, f"{t} parks on {r['name']}'s start"


def test_every_target_reachable_from_every_start(scenario):
    zones = [(x, y) for zs in scenario.zones().values() for x, y, _ in zs]
    secs = scenario.travel.matrix([(r['x'], r['y']) for r in scenario.robots], zones)
    assert all(math.isfinite(v) for row in secs for v in row)


def test_first_plan_rescues_everything(scenario):
    log = []
    mission = Mission([r['name'] for r in scenario.robots], scenario.targets, scenario.precedence,
                      scenario.travel, dataclasses.replace(PARAMS), log.append)
    mission.start(0.0)
    mission.tick(0.0, {r['name']: (r['x'], r['y'], r['yaw']) for r in scenario.robots})
    problem = mission.build_problem(0.0)
    assert mission.adopt(problem, solve_problem(problem), 0.0)
    plan = next(r for r in log if r['event'] == 'plan')
    assert plan['status'] in ('OPTIMAL', 'FEASIBLE')
    assert plan['mode'] == 'makespan' and not plan['skipped']
    assert plan['rescues'] == len(scenario.targets)
    assert plan['makespan'] < PARAMS.run_time_limit
