"""Draw every robot into the other robots' Nav2 costmaps.

The lidars scan about 0.155 m above the floor, above the ~0.09 m tall robot bodies, so in a scan
another robot is only its 7 cm lidar housing and Nav2 drives into parked robots. This node takes
each robot's map-frame pose (pose2d, published by robot_controller) and publishes two
OccupancyGrids with the arena map's geometry per robot, in which other robots are their body
rectangles (body_length x body_width + body_margin on every side, turned with the robot):

  /<robot>/other_robots   every other robot      -> local costmap (robots_layer)
  /<robot>/parked_robots  other robots standing   -> global costmap (robots_layer)

Both costmaps load their grid as a second static layer (config/nav2_params.yaml), so the inflation
layer keeps robots apart like walls. A rectangle instead of a 0.24 m disc matters next to a parked
robot: its flank is 0.16 m from its centre, so e.g. the second robot at the cooperative target C1
(midterm) has a 0.3 m wide lane between the parked robot and the wall instead of 0.15 m.

The global planner only gets the robots that stand still (moved less than `still_distance` within
`still_time`: parked, dwelling, waiting): a moving robot would close a corridor it is driving
through and the robot behind it could not plan to follow. still_time is longer than a turn on the
spot for the same reason. The local costmap has every robot, so the controller still stops before
driving into one.

A robot whose pose has not arrived yet is not drawn. A grid is republished only when the robots
drawn in it change or one of them moves by more than `min_move` or turns by more than `min_turn`.
A robot gets its grids only once its costmap has published itself
(/<robot>/{local,global}_costmap/costmap): a static layer that receives its first map while the
costmap is still being configured can crash the Nav2 container (seen once: segfault during the
global costmap's first resize).
"""
import array
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from edgeai_rescue.map_utils import load_map

OCCUPIED = 100


def robots_grid(shape, origin, resolution, poses, box, clear=None):
    """int8 grid (rows from the map origin up) with OCCUPIED in every cell whose centre lies inside
    one of the robots' body rectangles, 0 elsewhere. poses: (x, y, yaw) in map-frame metres/rad;
    box = (x_min, x_max, y_min, y_max): the body in the robot's own frame. clear = (x, y, r): cells
    within r of (x, y) stay free (the robot that receives the grid, see RobotOverlay.publish)."""
    h, w = shape
    x0, y0 = origin
    x_min, x_max, y_min, y_max = box
    reach = max(math.hypot(x, y) for x in (x_min, x_max) for y in (y_min, y_max))
    grid = np.zeros((h, w), dtype=np.int8)
    for x, y, yaw in poses:
        c0 = max(int(math.floor((x - reach - x0) / resolution)), 0)
        c1 = min(int(math.floor((x + reach - x0) / resolution)) + 1, w)
        r0 = max(int(math.floor((y - reach - y0) / resolution)), 0)
        r1 = min(int(math.floor((y + reach - y0) / resolution)) + 1, h)
        if c0 >= c1 or r0 >= r1:
            continue
        dx = (x0 + (np.arange(c0, c1) + 0.5) * resolution - x)[None, :]
        dy = (y0 + (np.arange(r0, r1) + 0.5) * resolution - y)[:, None]
        u = math.cos(yaw) * dx + math.sin(yaw) * dy      # cell centre in the robot's frame
        v = -math.sin(yaw) * dx + math.cos(yaw) * dy
        inside = (u >= x_min) & (u <= x_max) & (v >= y_min) & (v <= y_max)
        if clear is not None:
            inside &= (dx + x - clear[0]) ** 2 + (dy + y - clear[1]) ** 2 > clear[2] ** 2
        grid[r0:r1, c0:c1][inside] = OCCUPIED
    return grid


class StillTracker:
    """Which robots stand still: moved less than `distance` from where they were `time` ago."""

    def __init__(self, distance, time):
        self.distance, self.time = distance, time
        self.anchor = {}   # robot -> (x, y, t): where it has been within `distance` since t

    def update(self, robot, x, y, t):
        a = self.anchor.get(robot)
        if a is None or math.dist((x, y), a[:2]) > self.distance:
            self.anchor[robot] = (x, y, t)

    def still(self, robot, t):
        a = self.anchor.get(robot)
        return a is not None and t - a[2] >= self.time


class RobotOverlay(Node):
    def __init__(self):
        super().__init__('robot_overlay')
        self.robots = list(self.declare_parameter('robots', ['robot1', 'robot2']).value)
        map_yaml = self.declare_parameter('map_yaml', '').value
        # Body in base_link (edgeai_robot_real.urdf.xacro): plate 0.375 m long, centred on
        # base_link; 0.32 m wide over the wheels. The margin also covers the cells whose centre
        # falls just outside the rectangle (half a cell).
        length = float(self.declare_parameter('body_length', 0.375).value)
        width = float(self.declare_parameter('body_width', 0.32).value)
        margin = float(self.declare_parameter('body_margin', 0.03).value)
        self.box = (-length / 2 - margin, length / 2 + margin, -width / 2 - margin, width / 2 + margin)
        self.radius = math.hypot(length / 2 + margin, width / 2 + margin)   # farthest body point
        self.min_move = float(self.declare_parameter('min_move', 0.01).value)
        self.min_turn = float(self.declare_parameter('min_turn', 0.05).value)
        # A robot's own grid leaves the cells within clear_radius of it free (> the costmaps'
        # inscribed radius, 0.24 / 0.28 m): a robot that got too close to another one is never
        # inside an obstacle and can drive away. Without it two robots that stopped 0.47 m apart
        # blocked each other for good (scenario rooms). Driving closer is still caught by the
        # controller's look-ahead collision check against the rest of the other robot's body.
        self.clear_radius = float(self.declare_parameter('clear_radius', 0.35).value)
        self.tracker = StillTracker(float(self.declare_parameter('still_distance', 0.05).value),
                                    float(self.declare_parameter('still_time', 3.0).value))
        rate = float(self.declare_parameter('rate', 5.0).value)

        m = load_map(map_yaml)
        self.shape, self.origin, self.res = m['grid'].shape, m['origin'], m['resolution']
        self.poses = {}       # robot -> (x, y, yaw), map frame
        self.published = {}   # topic -> {robot: (x, y, yaw)} drawn in its last grid

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pubs = {}
        self.ready = set()    # (robot, topic) whose costmap is up
        self.waiting = {}     # (robot, topic) -> subscription to that costmap
        for r in self.robots:
            for kind, costmap in (('other_robots', 'local_costmap'), ('parked_robots', 'global_costmap')):
                self.pubs[r, kind] = self.create_publisher(OccupancyGrid, f'/{r}/{kind}', qos)
                self.waiting[r, kind] = self.create_subscription(
                    OccupancyGrid, f'/{r}/{costmap}/costmap', lambda _, key=(r, kind): self.on_costmap(key), qos)
            self.create_subscription(Pose2D, f'/{r}/pose2d', lambda msg, r=r: self.on_pose(r, msg), 10)
        self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info(f'drawing {", ".join(self.robots)} into each other\'s costmaps '
                               f'({self.box[1] - self.box[0]:.2f} x {self.box[3] - self.box[2]:.2f} m bodies on a '
                               f'{self.shape[1]} x {self.shape[0]} grid)')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_costmap(self, key):
        """The costmap is up: from now on it gets its grid (an empty one if no robot is drawn)."""
        if key in self.waiting:
            self.destroy_subscription(self.waiting.pop(key))
            self.ready.add(key)

    def on_pose(self, robot, msg):
        self.poses[robot] = (msg.x, msg.y, msg.theta)
        self.tracker.update(robot, msg.x, msg.y, self.now())

    def changed(self, key, drawn):
        last = self.published.get(key)
        if last is None or last.keys() != drawn.keys():
            return True
        return any(math.dist(drawn[o][:2], last[o][:2]) > self.min_move
                   or abs(math.remainder(drawn[o][2] - last[o][2], math.tau)) > self.min_turn for o in drawn)

    def publish(self):
        now = self.now()
        stamp = self.get_clock().now().to_msg()
        for r in self.robots:
            others = {o: p for o, p in self.poses.items() if o != r}
            parked = {o: p for o, p in others.items() if self.tracker.still(o, now)}
            me = self.poses.get(r)
            for kind, drawn in (('other_robots', others), ('parked_robots', parked)):
                # Only a robot closer than radius + clear_radius is cut, and only then the grid
                # depends on (and is republished for) the receiving robot's own position
                near = me is not None and any(math.dist(me[:2], p[:2]) < self.radius + self.clear_radius
                                              for p in drawn.values())
                key = dict(drawn, **({' self': me} if near else {}))
                if (r, kind) in self.ready and self.changed((r, kind), key):
                    clear = (*me[:2], self.clear_radius) if near else None
                    self.pubs[r, kind].publish(self.grid_msg(drawn.values(), stamp, clear))
                    self.published[r, kind] = key

    def grid_msg(self, poses, stamp, clear=None):
        grid = robots_grid(self.shape, self.origin, self.res, poses, self.box, clear)
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.info.map_load_time = stamp
        msg.info.resolution = self.res
        msg.info.height, msg.info.width = self.shape
        msg.info.origin.position.x, msg.info.origin.position.y = self.origin
        msg.info.origin.orientation.w = 1.0
        msg.data = array.array('b', grid.tobytes())
        return msg


def main():
    rclpy.init()
    node = RobotOverlay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():  # otherwise Ctrl-C shut the context down while spin was building its wait set
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
