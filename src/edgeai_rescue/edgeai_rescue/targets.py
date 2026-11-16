"""
Target list loading, rescue-zone geometry and ordering-rule validation. No ROS dependency.

Target file format (map frame, metres / radians):

    targets:
      - {id: S1, type: normal,      x: 1.0, y: 4.1, yaw: -1.5708, radius: 0.5}
      - {id: C1, type: cooperative, x: 4.1, y: 1.0, yaw:  1.5708, radius: 0.7}
    precedence: [[S1, C1]]   # optional ordering rules (Task 2B): S1 must be rescued before C1

x, y is the target reference position (the ground point beneath the marker stand), yaw is the
direction the marker faces, radius is the rescue radius around (x, y). An optional per-target
`dwell` overrides the coordinator's default dwell time.
"""
import math
from dataclasses import dataclass

import yaml

NORMAL = 'normal'
COOPERATIVE = 'cooperative'


@dataclass(frozen=True)
class Target:
    id: str
    type: str
    x: float
    y: float
    yaw: float
    radius: float = 0.5
    dwell: float | None = None  # None = use the coordinator's default for this type


def load_targets(path):
    """Targets only (kept for the old planner_node). See load_mission for the ordering rules."""
    return load_mission(path)[0]


def load_mission(path):
    """(targets, precedence) from a target file; precedence is a validated list of (id, id)."""
    with open(path) as f:
        data = yaml.safe_load(f)
    targets = []
    for e in data['targets']:
        kind = e['type'].lower()
        if kind not in (NORMAL, COOPERATIVE):
            raise ValueError(f"target {e['id']}: unknown type {e['type']!r}")
        dwell = e.get('dwell')
        targets.append(Target(str(e['id']), kind, float(e['x']), float(e['y']), float(e.get('yaw', 0.0)),
                              float(e.get('radius', 0.5)), None if dwell is None else float(dwell)))
    ids = [t.id for t in targets]
    if len(ids) != len(set(ids)):
        raise ValueError(f'duplicate target ids in {path}')
    precedence = validate_rules(data.get('precedence') or [], ids)
    return targets, precedence


def validate_rules(precedence, ids):
    """Checks ordering rules [(before, after), ...]: known ids, no self-loops, no cycles.

    Returns them as a list of (str, str) tuples; raises ValueError with the reason otherwise.
    """
    ids = set(ids)
    rules = []
    for rule in precedence:
        if not isinstance(rule, (list, tuple)) or len(rule) != 2:
            raise ValueError(f'rule {rule!r} is not a pair [before, after]')
        a, b = str(rule[0]), str(rule[1])
        for t in (a, b):
            if t not in ids:
                raise ValueError(f'rule {rule!r}: unknown target {t!r}')
        if a == b:
            raise ValueError(f'rule {rule!r}: a target cannot precede itself')
        if (a, b) not in rules:
            rules.append((a, b))
    # Kahn's algorithm: a cycle leaves nodes with incoming edges
    in_degree = {t: 0 for t in ids}
    for _, b in rules:
        in_degree[b] += 1
    ready = [t for t, d in in_degree.items() if d == 0]
    seen = 0
    while ready:
        t = ready.pop()
        seen += 1
        for a, b in rules:
            if a == t:
                in_degree[b] -= 1
                if in_degree[b] == 0:
                    ready.append(b)
    if seen != len(ids):
        raise ValueError('ordering rules contain a cycle')
    return rules


def zone_points(target, normal_offset=0.40, coop_offset=0.45, coop_spacing=0.6):
    """Parking poses (x, y, yaw) where robots stop to rescue a target: 1 for normal, 2 for cooperative.

    The rescue counts when the robot centre is within target.radius of (x, y), but the stand
    occupies that point, so robots park in front of the marker (along its facing direction) and
    face it. Cooperative targets get two spots side by side, coop_spacing apart. This is the one
    place where the marker-to-zone rule lives (the brief's zone_from_marker).

    Midterm values: S1 (r=0.5) -> 0.40 m from the centre (closer puts the parked robot inside
    the stand's inflation in Nav2's costmap, and it cannot plan away again); C1 (r=0.7) -> 0.45 ahead, +-0.30
    sideways = 0.54 m from the centre, 0.6 m between the robots.
    """
    fx, fy = math.cos(target.yaw), math.sin(target.yaw)
    face = math.atan2(-fy, -fx)  # robots look back at the marker
    if target.type == NORMAL:
        return [(target.x + normal_offset * fx, target.y + normal_offset * fy, face)]
    cx, cy = target.x + coop_offset * fx, target.y + coop_offset * fy
    half = coop_spacing / 2
    return [(cx - half * fy, cy + half * fx, face), (cx + half * fy, cy - half * fx, face)]


def rescue_zones(target, offset, coop_spacing=0.7):
    """Zone centres (x, y) for the old planner_node: `offset` in front of the marker for both types."""
    return [(x, y) for x, y, _ in zone_points(target, offset, offset, coop_spacing)]
