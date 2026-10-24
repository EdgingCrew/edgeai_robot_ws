"""Load a saved Nav2/SLAM Toolbox map and compute A* travel distances on it.

Works without ROS (only numpy and PyYAML), so it can run offline or inside a node.
Grid convention matches nav_msgs/OccupancyGrid: row 0 is the bottom of the map
(lowest y), values are 0 = free, 100 = occupied, -1 = unknown.
"""
import heapq
import math
import os

import numpy as np
import yaml


def load_map(yaml_path):
    """Read a map saved by map_saver_cli (YAML + PGM), applying Nav2's thresholds."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    image_path = os.path.join(os.path.dirname(yaml_path), meta["image"])
    pixels = read_pgm(image_path).astype(float)

    # Nav2 map_server rule: p = how occupied a pixel is, from 0 (white) to 1 (black).
    p = pixels / 255.0 if meta.get("negate", 0) else (255.0 - pixels) / 255.0
    grid = np.full(p.shape, -1, dtype=np.int8)
    grid[p > meta["occupied_thresh"]] = 100
    grid[p < meta["free_thresh"]] = 0

    x0, y0, yaw = meta["origin"]
    if abs(yaw) > 1e-6:
        raise ValueError("rotated map origins are not supported")
    # Image row 0 is the top of the map; OccupancyGrid row 0 is the bottom.
    return {"grid": np.flipud(grid), "resolution": float(meta["resolution"]), "origin": (x0, y0)}


def from_occupancy_grid(msg):
    """Same structure from a live nav_msgs/OccupancyGrid message (e.g. the /map topic)."""
    info = msg.info
    grid = np.array(msg.data, dtype=np.int8).reshape(info.height, info.width)
    return {"grid": grid, "resolution": info.resolution,
            "origin": (info.origin.position.x, info.origin.position.y)}


def read_pgm(path):
    """Minimal reader for binary (P5) and text (P2) 8-bit PGM files."""
    with open(path, "rb") as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:  # magic, width, height, maxval; skip '#' comments
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    magic, w, h = tokens[0], int(tokens[1]), int(tokens[2])
    if magic == b"P5":
        return np.frombuffer(data[pos + 1:pos + 1 + w * h], dtype=np.uint8).reshape(h, w)
    if magic == b"P2":
        return np.array(data[pos:].split()[:w * h], dtype=np.uint8).reshape(h, w)
    raise ValueError(f"unsupported image format {magic!r}")


def world_to_cell(m, x, y):
    """Map-frame metres -> (row, col)."""
    x0, y0 = m["origin"]
    return int(math.floor((y - y0) / m["resolution"])), int(math.floor((x - x0) / m["resolution"]))


def cell_to_world(m, row, col):
    """(row, col) -> map-frame metres at the cell centre."""
    x0, y0 = m["origin"]
    return x0 + (col + 0.5) * m["resolution"], y0 + (row + 0.5) * m["resolution"]


def blocked_cells(m, robot_radius, unknown_is_blocked=True):
    """Cells the robot centre may not enter: obstacles grown by the robot radius."""
    grid = m["grid"]
    obstacle = (grid == 100) | ((grid == -1) if unknown_is_blocked else False)
    r = int(math.ceil(robot_radius / m["resolution"]))
    blocked = obstacle.copy()
    h, w = grid.shape
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if dr * dr + dc * dc > r * r:
                continue
            src = obstacle[max(0, -dr):h - max(0, dr), max(0, -dc):w - max(0, dc)]
            blocked[max(0, dr):h - max(0, -dr), max(0, dc):w - max(0, -dc)] |= src
    return blocked


def astar(blocked, start, goal, resolution):
    """8-connected A* on the grid. Returns (length in metres, list of cells) or (inf, [])."""
    h, w = blocked.shape
    if blocked[start] or blocked[goal]:
        return math.inf, []
    moves = [(dr, dc, math.hypot(dr, dc)) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]

    def heuristic(cell):  # octile distance: exact on an empty 8-connected grid
        dr, dc = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
        return max(dr, dc) + (math.sqrt(2) - 1) * min(dr, dc)

    g = {start: 0.0}
    parent = {start: None}
    frontier = [(heuristic(start), start)]
    while frontier:
        _, cell = heapq.heappop(frontier)
        if cell == goal:
            path = []
            while cell is not None:
                path.append(cell)
                cell = parent[cell]
            return g[goal] * resolution, path[::-1]
        r, c = cell
        for dr, dc, step in moves:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w) or blocked[nr, nc]:
                continue
            if dr and dc and (blocked[r, nc] or blocked[nr, c]):
                continue  # no cutting corners past an obstacle
            cost = g[cell] + step
            if cost < g.get((nr, nc), math.inf):
                g[(nr, nc)] = cost
                parent[(nr, nc)] = cell
                heapq.heappush(frontier, (cost + heuristic((nr, nc)), (nr, nc)))
    return math.inf, []


def nearest_free(blocked, cell):
    """Closest non-blocked cell (breadth-first), for goals that fall inside the inflation."""
    from collections import deque
    h, w = blocked.shape
    seen, queue = {cell}, deque([cell])
    while queue:
        r, c = queue.popleft()
        if 0 <= r < h and 0 <= c < w and not blocked[r, c]:
            return (r, c)
        for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if (nr, nc) not in seen and 0 <= nr < h and 0 <= nc < w:
                seen.add((nr, nc))
                queue.append((nr, nc))
    return None


def draw_map(m, path=(), points=(), blocked=None, ax=None):
    """Plot the grid, optional inflation, an A* path (cells) and points ((x, y) in metres).

    Drawn in map-frame metres. Returns the matplotlib axes; call plt.show() or savefig.
    """
    import matplotlib.pyplot as plt  # only needed here, so the rest stays matplotlib-free
    from matplotlib.colors import ListedColormap

    if ax is None:
        _, ax = plt.subplots()
    grid = m["grid"]
    h, w = grid.shape
    x0, y0 = m["origin"]
    res = m["resolution"]
    extent = (x0, x0 + w * res, y0, y0 + h * res)

    # Same shading as RViz/map_saver: free white, unknown grey, occupied black.
    shade = np.full(grid.shape, 0.6)
    shade[grid == 0] = 1.0
    shade[grid == 100] = 0.0
    ax.imshow(shade, cmap="gray", vmin=0, vmax=1, origin="lower", extent=extent,
              interpolation="nearest")
    if blocked is not None:
        inflation = np.ma.masked_where(~blocked | (grid == 100), blocked)
        ax.imshow(inflation, cmap=ListedColormap(["tab:red"]), alpha=0.3, origin="lower", extent=extent,
                  interpolation="nearest")

    if len(path):
        xs, ys = zip(*(cell_to_world(m, r, c) for r, c in path))
        ax.plot(xs, ys, "-", color="tab:blue", linewidth=2, label="path")
    for i, (x, y) in enumerate(points):
        ax.plot(x, y, "o", markersize=8, label=f"point {i}")

    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    if len(path) or len(points):
        ax.legend(loc="best")
    return ax


if __name__ == "__main__":
    import sys
    m = load_map(sys.argv[1])
    blocked = blocked_cells(m, robot_radius=0.23)
    grid = m["grid"]
    print(f"{grid.shape[1]} x {grid.shape[0]} cells at {m['resolution']} m, origin {m['origin']}")
    print(f"free {np.sum(grid == 0)}, occupied {np.sum(grid == 100)}, unknown {np.sum(grid == -1)}")
    a = nearest_free(blocked, world_to_cell(m, float(sys.argv[2]), float(sys.argv[3])))
    b = nearest_free(blocked, world_to_cell(m, float(sys.argv[4]), float(sys.argv[5])))
    dist, path = astar(blocked, a, b, m["resolution"])
    print(f"A* from {cell_to_world(m, *a)} to {cell_to_world(m, *b)}: {dist:.2f} m, {len(path)} cells")
