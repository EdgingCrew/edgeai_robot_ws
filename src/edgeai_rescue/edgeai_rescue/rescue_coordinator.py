"""
Rescue coordinator: CP-SAT task allocation for Task 2, executed through each robot's controller.

The decisions live in mission.Mission (no ROS). This node only connects it to ROS:
  * subscribes /<robot>/pose2d (map-frame pose from each robot_controller, used for dwell checks);
  * sends navigate-only ExecuteRescue goals (dwell_time = 0) to /<robot>/execute_rescue;
  * runs the solver in a worker thread so the 10 Hz tick (dwell timing) never blocks;
  * /start_rescue (std_srvs/Empty) starts a run, /rescue_rules (std_msgs/String, JSON
    {"precedence": [["S1", "C1"]]} or just [["S1", "C1"]]) replaces the ordering rules (Task 2B);
  * writes the JSONL run log (logs/task2_<timestamp>.jsonl) and publishes /target_markers.
"""
import concurrent.futures
import dataclasses
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose2D
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

from edgeai_rescue.map_utils import load_map
from edgeai_rescue.mission import DONE, DROPPED, DWELLING, Mission, MissionParams, solve_problem
from edgeai_rescue.run_log import RunLog
from edgeai_rescue.scheduler.travel import TravelMatrix
from edgeai_rescue.targets import load_mission, validate_rules
from edgeai_rescue_interfaces.action import ExecuteRescue

QUIET_EVENTS = {'distance'}  # logged to file only


class RescueCoordinator(Node):

    def __init__(self):
        super().__init__('rescue_coordinator')
        self.robot_names = list(self.declare_parameter('robot_names', ['robot1', 'robot2']).value)
        targets_file = self.declare_parameter('targets_file', '').value
        map_yaml = self.declare_parameter('map_yaml', '').value
        robot_radius = float(self.declare_parameter('robot_radius', 0.24).value)
        nominal_speed = float(self.declare_parameter('nominal_speed', 0.25).value)
        goal_overhead = float(self.declare_parameter('goal_overhead', 1.0).value)
        self.log_dir = self.declare_parameter('log_dir', 'logs').value
        self.autostart = bool(self.declare_parameter('autostart', False).value)
        tick_rate = float(self.declare_parameter('tick_rate', 10.0).value)
        self.params = MissionParams(**{
            f.name: type(f.default)(self.declare_parameter(f.name, f.default).value)
            for f in dataclasses.fields(MissionParams)})

        self.targets, self.precedence = load_mission(targets_file)
        self.travel = TravelMatrix(load_map(map_yaml), robot_radius, nominal_speed, goal_overhead)
        self.get_logger().info(f'{len(self.targets)} targets from {targets_file}, map {map_yaml}, '
                               f'scheduler {self.params.scheduler}, mode {self.params.mode}')
        # Distance fields and the blocking analysis are cached in self.travel for every run
        t0 = time.monotonic()
        cut = Mission(self.robot_names, self.targets, self.precedence, self.travel, self.params).precompute()
        spots = sorted({z for z, _, _ in cut})
        self.get_logger().info(f'travel times ready in {time.monotonic() - t0:.1f} s; {len(spots)} parking spots '
                               f'block other legs when occupied ({len(cut)} legs)')

        self.cb = ReentrantCallbackGroup()
        self.poses = {}
        for name in self.robot_names:
            self.create_subscription(Pose2D, f'/{name}/pose2d',
                                     lambda msg, n=name: self.poses.__setitem__(n, (msg.x, msg.y, msg.theta)),
                                     10, callback_group=self.cb)
        self.rescue_clients = {n: ActionClient(self, ExecuteRescue, f'/{n}/execute_rescue', callback_group=self.cb)
                        for n in self.robot_names}
        self.active = {n: None for n in self.robot_names}  # robot -> (seq, goal handle) of the running goal
        self.pending = {}                                    # robot -> latest unsent command
        self.dispatching = {n: False for n in self.robot_names}

        self.marker_pub = self.create_publisher(MarkerArray, 'target_markers', 10)
        self.create_service(Empty, 'start_rescue', self.start_callback, callback_group=self.cb)
        self.create_subscription(String, 'rescue_rules', self.rules_callback, 10, callback_group=self.cb)

        self.mission = None
        self.run_log = None
        self.start_requested = self.autostart
        self.pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.solve_future, self.problem = None, None
        self.create_timer(1.0 / tick_rate, self.tick, callback_group=self.cb)
        self.create_timer(0.5, self.publish_markers, callback_group=self.cb)

    # ------------------------------------------------------------ helpers
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def record(self, rec):
        if self.run_log is not None:
            self.run_log.write(rec)
        if rec['event'] in QUIET_EVENTS:
            return
        brief = {k: v for k, v in rec.items() if k not in ('t', 'event', 'routes', 'params', 'targets')}
        self.get_logger().info(f"[{rec['t']:7.2f}] {rec['event']}: {json.dumps(brief, default=str)}")
        if 'routes' in rec:
            for robot, route in rec['routes'].items():
                self.get_logger().info(f'    {robot}: ' + (' -> '.join(
                    f"{v['task']}[{v['slot']}] @{v['start']:.1f}-{v['end']:.1f}" for v in route) or 'idle'))

    # ------------------------------------------------------------ start / rules
    def start_callback(self, request, response):
        self.start_requested = True
        self.get_logger().info('Start requested.')
        return response

    def try_start(self):
        missing = [n for n in self.robot_names if n not in self.poses]
        if missing or self.now() <= 0.0:
            self.get_logger().info(f'Waiting for poses of {missing} / sim clock before starting...',
                                   throttle_duration_sec=5.0)
            return
        for name in self.robot_names:
            self.queue_command(('cancel', name))
        if self.run_log is not None:
            self.run_log.close()
        self.run_log = RunLog(self.log_dir)
        self.get_logger().info(f'Logging to {self.run_log.path}')
        self.mission = Mission(self.robot_names, self.targets, self.precedence, self.travel, self.params, self.record)
        self.solve_future, self.problem = None, None
        self.mission.start(self.now())
        self.start_requested = False

    def rules_callback(self, msg):
        try:
            data = json.loads(msg.data)
            rules = data['precedence'] if isinstance(data, dict) else data
            rules = validate_rules(rules, [t.id for t in self.targets])
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().error(f'Ignoring invalid ordering rules {msg.data!r}: {e}')
            if self.mission is not None:
                self.mission.log('rules_rejected', raw=msg.data, error=str(e))
            return
        self.precedence = rules
        if self.mission is not None:
            self.mission.set_precedence(rules)
        else:
            self.get_logger().info(f'Ordering rules set to {rules} for the next run.')

    # ------------------------------------------------------------ main loop
    def tick(self):
        if self.start_requested:
            self.try_start()
        m = self.mission
        if m is None:
            return
        now = self.now()
        m.tick(now, self.poses)
        if self.solve_future is not None and self.solve_future.done():
            future, problem = self.solve_future, self.problem
            self.solve_future, self.problem = None, None
            try:
                m.adopt(problem, future.result(), now)
            except Exception as e:  # noqa: BLE001 - a solver crash must not kill the run
                self.get_logger().error(f'Solver failed: {e!r}')
                m.abort_solve(repr(e))
        elif self.solve_future is None and m.replan_due(now):
            problem = m.build_problem(now)
            if problem is not None:
                self.problem = problem
                self.solve_future = self.pool.submit(solve_problem, problem)
        for cmd in m.take_commands():
            self.queue_command(cmd)

    # ------------------------------------------------------------ goal dispatch (one worker per robot)
    def queue_command(self, cmd):
        robot = cmd[1]
        self.pending[robot] = cmd  # only the newest command matters
        if not self.dispatching[robot]:
            self.dispatching[robot] = True
            self.executor.create_task(self.dispatch_loop(robot))

    async def dispatch_loop(self, robot):
        try:
            while robot in self.pending:
                cmd = self.pending.pop(robot)
                await self.cancel_active(robot)
                if cmd[0] == 'goal':
                    await self.send_goal(*cmd[1:])
        finally:
            self.dispatching[robot] = False

    async def cancel_active(self, robot):
        """Cancel the robot's running goal and wait until the controller has released it."""
        active, self.active[robot] = self.active[robot], None
        if active is None:
            return
        _, handle = active
        await handle.cancel_goal_async()
        await handle.get_result_async()

    async def send_goal(self, robot, seq, x, y, yaw, task):
        client = self.rescue_clients[robot]
        mission = self.mission  # results go to the run that sent the goal, not a restarted one
        if not client.server_is_ready():
            self.get_logger().error(f'execute_rescue server of {robot} is not available')
            mission.on_nav_result(robot, seq, False, self.now())
            return
        goal = ExecuteRescue.Goal()
        goal.target_id = f'{task}'
        goal.zone = Pose2D(x=float(x), y=float(y), theta=float(yaw))
        goal.tolerance = 0.0
        goal.dwell_time = 0.0  # navigate only; the coordinator times the dwell
        handle = await client.send_goal_async(goal)
        if not handle.accepted:
            self.get_logger().error(f'{robot} rejected the goal for {task}')
            mission.on_nav_result(robot, seq, False, self.now())
            return
        self.active[robot] = (seq, handle)
        self.executor.create_task(self.watch_result(mission, robot, seq, handle))

    async def watch_result(self, mission, robot, seq, handle):
        result = await handle.get_result_async()
        if self.active[robot] is not None and self.active[robot][1] is handle:
            self.active[robot] = None
        if mission is self.mission:
            mission.on_nav_result(robot, seq, result.status == GoalStatus.STATUS_SUCCEEDED, self.now())

    # ------------------------------------------------------------ RViz
    def publish_markers(self):
        arr = MarkerArray()
        m = self.mission
        for i, target in enumerate(self.targets):
            task = m.tasks[target.id] if m else None
            status = task.status if task else 'OPEN'
            dwelling = task is not None and task.dwell_start is not None
            busy = m is not None and any(r.queue and r.queue[0].task == target.id for r in m.robots.values())
            if status == DONE:
                rgb = (0.0, 0.8, 0.0)
            elif status == DROPPED:
                rgb = (0.4, 0.4, 0.4)
            elif dwelling:
                rgb = (1.0, 0.9, 0.0)
            elif busy:
                rgb = (1.0, 0.5, 0.0)
            else:
                rgb = (0.9, 0.0, 0.0)

            disc = self.marker('rescue_radius', i, Marker.CYLINDER, target.x, target.y, 0.005, rgb, 0.3)
            disc.scale.x = disc.scale.y = 2 * target.radius
            disc.scale.z = 0.01
            arr.markers.append(disc)

            arrow = self.marker('marker_facing', i, Marker.ARROW, target.x, target.y, 0.35, rgb, 1.0)
            arrow.pose.orientation.z, arrow.pose.orientation.w = math.sin(target.yaw / 2), math.cos(target.yaw / 2)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.3, 0.05, 0.05
            arr.markers.append(arrow)

            spots = self.marker('parking', i, Marker.SPHERE_LIST, 0.0, 0.0, 0.05, (0.2, 0.4, 1.0), 0.9)
            spots.scale.x = spots.scale.y = spots.scale.z = 0.08
            for x, y, _ in (task.zones if task else []):
                spots.points.append(Point(x=x, y=y, z=0.05))
            if spots.points:
                arr.markers.append(spots)

            text = f'{target.id} {status}'
            if dwelling:
                text += f' {m.now - task.dwell_start:.1f}/{task.dwell:.0f}s'
            elif status == DONE:
                text += f" by {','.join(task.robots)} @{task.rescued_at:.1f}s"
            label = self.marker('labels', i, Marker.TEXT_VIEW_FACING, target.x, target.y, 0.7, (1.0, 1.0, 1.0), 1.0)
            label.scale.z = 0.18
            label.text = text
            arr.markers.append(label)
        self.marker_pub.publish(arr)

    def marker(self, ns, i, kind, x, y, z, rgb, alpha):
        mk = Marker()
        mk.header.frame_id = 'map'
        mk.ns, mk.id, mk.type, mk.action = ns, i, kind, Marker.ADD
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(x), float(y), float(z)
        mk.pose.orientation.w = 1.0
        mk.color.r, mk.color.g, mk.color.b = rgb
        mk.color.a = alpha
        return mk

    def destroy_node(self):
        self.pool.shutdown(wait=False, cancel_futures=True)
        if self.run_log is not None:
            self.run_log.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = RescueCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
