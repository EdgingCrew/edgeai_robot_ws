from edgeai_rescue.map_utils import load_map, blocked_cells, world_to_cell, astar, draw_map
import matplotlib.pyplot as plt

m = load_map("maps/midterm_arena.yaml")
blocked = blocked_cells(m, robot_radius=0.24)
rx, ry = 0.45, 0.5  # robot1 start (world = map frame)
tx, ty = 1.0, 3.70  # S1 parking point
dist, path = astar(blocked, world_to_cell(m, rx, ry), world_to_cell(m, tx, ty), m["resolution"])
draw_map(m, path, points=[(rx, ry), (tx, ty)], blocked=blocked)
print(f"Distance from ({rx}, {ry}) to ({tx}, {ty}) is {dist:.2f} m")
plt.show()