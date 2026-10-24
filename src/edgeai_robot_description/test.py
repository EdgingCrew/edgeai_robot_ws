from map_utils import load_map, blocked_cells, world_to_cell, astar, draw_map
import matplotlib.pyplot as plt

m = load_map("maps/edgeai_arena.yaml")
blocked = blocked_cells(m, robot_radius=0.23)
rx, ry = 0.0, 0.0  # robot position in world coordinates
tx, ty = 3.0, 3.0  # target position in world coordinates
dist, path = astar(blocked, world_to_cell(m, rx, ry), world_to_cell(m, tx, ty), m["resolution"])
draw_map(m, path, points=[(rx, ry), (tx, ty)], blocked=blocked)
print(f"Distance from ({rx}, {ry}) to ({tx}, {ty}) is {dist:.2f} m")
plt.show()