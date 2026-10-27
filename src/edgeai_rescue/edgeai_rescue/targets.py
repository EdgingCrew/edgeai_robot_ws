"""
Target list loading and rescue-zone geometry.
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


def load_targets(path):
    with open(path) as f:
        entries = yaml.safe_load(f)['targets']
    targets = []
    for e in entries:
        kind = e['type'].lower()
        if kind not in (NORMAL, COOPERATIVE):
            raise ValueError(f"target {e['id']}: unknown type {e['type']!r}")
        targets.append(Target(str(e['id']), kind, float(e['x']), float(e['y']), float(e.get('yaw', 0.0))))
    ids = [t.id for t in targets]
    if len(ids) != len(set(ids)):
        raise ValueError(f'duplicate target ids in {path}')
    return targets


def rescue_zones(target, offset, coop_spacing=0.7):
    """
    Calculates rescue zone centres (x, y) in the map frame: 1 for normal, 2 for cooperative targets.

    Placeholder for the official rule (still TBD): zones lie `offset` metres in front of the
    marker, facing it; the two cooperative zones are `coop_spacing` apart, side by side.
    """
    fx, fy = math.cos(target.yaw), math.sin(target.yaw)
    cx, cy = target.x + offset * fx, target.y + offset * fy
    if target.type == NORMAL:
        return [(cx, cy)]
    # Sideways unit vector is the marker normal rotated by 90 degrees
    half = coop_spacing / 2
    return [(cx - half * fy, cy + half * fx), (cx + half * fy, cy - half * fx)]
