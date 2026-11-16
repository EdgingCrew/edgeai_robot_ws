"""Travel-time matrix for the scheduler, from the saved occupancy map. No ROS dependency.

Distances come from one 8-connected Dijkstra per source point over the map with obstacles (and
unknown cells) grown by the robot radius, the same preparation as map_utils' A*. Points that fall
inside the inflation are snapped to the nearest free cell.

    tm = TravelMatrix(load_map("midterm_arena.yaml"), robot_radius=0.24, nominal_speed=0.25)
    seconds = tm.matrix(robot_xy, zone_xy)   # nodes: robots first, then zones; inf = unreachable

Distances from zone points are cached (zones never move); robot rows are recomputed every call.

blocking() finds the legs that a robot parked on a zone cuts off (a target inside a corridor or a
doorway): the scheduler must not send another robot along such a leg while the zone is occupied.
"""
import heapq
import math

import numpy as np

from edgeai_rescue.map_utils import blocked_cells, nearest_free, world_to_cell

SQRT2 = math.sqrt(2.0)


class TravelMatrix:
    def __init__(self, grid_map, robot_radius, nominal_speed, goal_overhead=1.0):
        if nominal_speed <= 0:
            raise ValueError("nominal_speed must be positive")
        self.map = grid_map
        self.robot_radius = float(robot_radius)
        self.res = grid_map["resolution"]
        self.blocked = blocked_cells(grid_map, robot_radius, unknown_is_blocked=True)
        self.h, self.w = self.blocked.shape
        self.speed = float(nominal_speed)
        self.overhead = float(goal_overhead)
        self._cache = {}  # snapped cell -> distance field (metres per cell)
        self._cut_cache = {}  # (zone cell, target cell, clearance) -> distance field with the zone occupied
        self._split_cache = {}  # (zone cell, clearance) -> does an occupied zone split the free space around it

    def snap(self, x, y):
        """Nearest free (row, col) to a map-frame point, or None if the map has no free cell."""
        r, c = world_to_cell(self.map, x, y)
        r = min(max(r, 0), self.h - 1)
        c = min(max(c, 0), self.w - 1)
        return nearest_free(self.blocked, (r, c))

    def distance_field(self, cell):
        """Metres from `cell` to every cell (inf where unreachable), 8-connected, no corner cutting."""
        if cell not in self._cache:
            self._cache[cell] = _dijkstra(self.blocked, cell) * self.res
        return self._cache[cell]

    def metres(self, a, b):
        """Path length in metres between two map-frame points (inf if unreachable)."""
        ca, cb = self.snap(*a), self.snap(*b)
        if ca is None or cb is None:
            return math.inf
        return float(self.distance_field(cb)[ca])

    def seconds(self, metres):
        return metres / self.speed + self.overhead

    def matrix(self, robot_xy, zone_xy):
        """Travel times in seconds between all nodes: robots first, then zones.

        Diagonal is 0; unreachable pairs are inf. Zone fields are cached, robot fields are not.
        """
        points = list(robot_xy) + list(zone_xy)
        n_r = len(robot_xy)
        cells = [self.snap(x, y) for x, y in points]
        fields = []
        for i, cell in enumerate(cells):
            if cell is None:
                fields.append(None)
            elif i < n_r:
                fields.append(self.distance_field(cell))
                self._cache.pop(cell, None)  # robot positions change; keep the cache to zones
            else:
                fields.append(self.distance_field(cell))
        n = len(points)
        out = [[0.0] * n for _ in range(n)]
        for u in range(n):
            for v in range(n):
                if u == v:
                    continue
                if fields[u] is None or cells[v] is None:
                    out[u][v] = math.inf
                    continue
                d = float(fields[u][cells[v]])
                out[u][v] = self.seconds(d) if math.isfinite(d) else math.inf
        return out

    # ------------------------------------------------------------ blocking by parked robots
    def blocking(self, robot_xy, zone_xy, zone_group, clearance, min_detour):
        """Legs cut off by a robot standing on a node, as (z, u, w) node indices (numbering as in matrix()).

        While a robot stands on node z (a zone it rescues from, or a robot's current position), the leg
        u -> w (w a zone) is impossible or at least min_detour metres longer. clearance = parked robot's
        radius + planning radius (how close the moving robot's centre can get). zone_group[i] is shared
        by the zones of one task: the two spots of a cooperative target never block each other's legs.
        Only nodes that split the free space around them (corridors, doorways) are analysed; the
        fields are cached per blocking cell and target, so repeated calls are cheap.
        """
        n_r = len(robot_xy)
        cells = [self.snap(x, y) for x, y in list(robot_xy) + list(zone_xy)]
        group = [None] * n_r + list(zone_group)
        out = []
        for z, z_cell in enumerate(cells):
            if z_cell is None or not self._splits(z_cell, clearance):
                continue
            cut_blocked = self.blocked | self._disc(z_cell, clearance)
            for w in range(n_r, len(cells)):
                w_cell = cells[w]
                if w == z or w_cell is None or (group[z] is not None and group[w] == group[z]):
                    continue
                base = self.distance_field(w_cell)
                key = (z_cell, w_cell, clearance)
                if key not in self._cut_cache:
                    start = w_cell if not cut_blocked[w_cell] else nearest_free(cut_blocked, w_cell)
                    self._cut_cache[key] = (np.full(self.blocked.shape, math.inf) if start is None
                                            else _dijkstra(cut_blocked, start) * self.res)
                cut = self._cut_cache[key]
                for u, u_cell in enumerate(cells):
                    if u in (z, w) or u_cell is None or (group[z] is not None and group[u] == group[z]):
                        continue
                    d0 = base[u_cell]
                    if not math.isfinite(d0):
                        continue
                    src = u_cell if not cut_blocked[u_cell] else nearest_free(cut_blocked, u_cell)
                    d1 = cut[src] if src is not None else math.inf
                    if d1 - d0 > min_detour:
                        out.append((z, u, w))
        return out

    def _disc(self, cell, radius):
        rr, cc = np.ogrid[:self.h, :self.w]
        return (rr - cell[0]) ** 2 + (cc - cell[1]) ** 2 <= (radius / self.res) ** 2

    def _splits(self, cell, clearance):
        """Does a robot on `cell` split the free cells right around it into separate parts?

        Looks at a window of clearance + 0.5 m: the free cells just outside the occupied disc are
        labelled by connectivity inside the window; more than one label means a corridor or doorway.
        """
        key = (cell, clearance)
        if key in self._split_cache:
            return self._split_cache[key]
        reach = int(math.ceil((clearance + 0.5) / self.res))
        r0, r1 = max(cell[0] - reach, 0), min(cell[0] + reach + 1, self.h)
        c0, c1 = max(cell[1] - reach, 0), min(cell[1] + reach + 1, self.w)
        rr, cc = np.ogrid[r0:r1, c0:c1]
        d2 = (rr - cell[0]) ** 2 + (cc - cell[1]) ** 2
        inner = (clearance / self.res) ** 2
        free = ~self.blocked[r0:r1, c0:c1] & (d2 > inner) & (d2 <= reach ** 2)
        ring = free & (d2 <= (clearance / self.res + 2) ** 2)
        labels = np.zeros(free.shape, dtype=int)
        n_labels = 0
        for start in zip(*np.nonzero(ring)):
            if labels[start]:
                continue
            n_labels += 1
            if n_labels > 1:
                break
            labels[start] = n_labels
            stack = [start]
            while stack:
                r, c = stack.pop()
                for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                    if 0 <= nr < free.shape[0] and 0 <= nc < free.shape[1] and free[nr, nc] and not labels[nr, nc]:
                        labels[nr, nc] = n_labels
                        stack.append((nr, nc))
        self._split_cache[key] = n_labels > 1
        return n_labels > 1


def _dijkstra(blocked, cell):
    """Grid steps (1 or sqrt 2) from `cell` to every cell; 8-connected, no corner cutting."""
    h, w = blocked.shape
    dist = np.full((h, w), math.inf)
    dist[cell] = 0.0
    heap = [(0.0, cell[0], cell[1])]
    moves = [(dr, dc, SQRT2 if dr and dc else 1.0) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
    while heap:
        d, r, c = heapq.heappop(heap)
        if d > dist[r, c]:
            continue
        for dr, dc, step in moves:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w) or blocked[nr, nc]:
                continue
            if dr and dc and (blocked[r, nc] or blocked[nr, c]):
                continue
            nd = d + step
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                heapq.heappush(heap, (nd, nr, nc))
    return dist
