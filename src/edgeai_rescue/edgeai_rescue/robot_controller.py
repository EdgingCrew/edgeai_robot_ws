"""
Robot controller node for the edgeAI robot.
"""
import math
from platform import node

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.task import Future
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from tf2_ros import Buffer, TransformException, TransformListener

from edgeai_rescue_interfaces.action import ExecuteRescue
from edgeai_rescue_interfaces.srv import EstimateCosts

def quaternion_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller_node")
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.period = self.declare_parameter('period', 0.5).value
        self.robot = self.get_namespace().strip("/") or 'robot'

        self.target_id = ''
        self.cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.rescue_server = ActionServer(self, ExecuteRescue, "execute_rescue", 
                                          execute_callback=self.execute_rescue_callback, goal_callback=self.rescue_goal_callback, 
                                          cancel_callback= lambda _: CancelResponse.ACCEPT,
                                          callback_group=self.cb)
        self.create_service(EstimateCosts, "estimate_costs", self.estimate_costs_callback, callback_group=self.cb)
        self.get_logger().info(f"[RobotController-{self.robot}]: Initialized!")

    async def sleep(self, seconds):
        """Non-blocking sleep in node time (sim time with use_sim_time)."""
        done = Future()

        def fire():
            timer.cancel()
            done.set_result(None)
        timer = self.create_timer(seconds, fire, callback_group=self.cb)
        await done
        self.destroy_timer(timer)

    def pose(self):
        """(x, y, yaw) of the robot in the global frame, or None while TF is not available."""
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, quaternion_yaw(t.transform.rotation)

    def stop(self):
        """Stop the robot by publishing a zero velocity command."""
        self.vel_pub.publish(Twist())
    
    def pose_stamped(self, pose2d):
        msg = PoseStamped()
        msg.header.frame_id = self.global_frame  # zero stamp = use the latest transform
        msg.pose.position.x = pose2d.x
        msg.pose.position.y = pose2d.y
        msg.pose.orientation.z = math.sin(pose2d.theta / 2.0)
        msg.pose.orientation.w = math.cos(pose2d.theta / 2.0)
        return msg

    # estimate_costs 
    async def estimate_costs_callback(self, request, response):
        """Estimate the costs of navigating to a set of poses."""
        response.costs = [await self.path_length(goal) for goal in request.goals]
        return response

    async def path_length(self, pose):
        if not self.path_client.is_ready():
            return math.inf
        goal = ComputePathToPose.Goal()
        goal.pose = pose
        goal.use_start = True
        handle = await self.path_client.send_goal_async(goal)
        if not handle.accepted:
            return math.inf
        result = await handle.get_result_async()
        poses = result.result.path.poses
        if result.status != GoalStatus.STATUS_SUCCEEDED or not poses:
            return math.inf
        return sum(math.dist((a.pose.position.x, a.pose.position.y), (b.pose.position.x, b.pose.position.y)) for a, b in zip(poses, poses[1:]))

    # execute_rescue
    async def rescue_goal_callback(self, goal_request):
        if self.target_id != '' and self.target_id != goal_request.target_id:
            self.get_logger().warn(f"Rejecting rescue goal for target {goal_request.target_id}, currently rescuing {self.target_id}")
            return GoalResponse.REJECT
        self.target_id = goal_request.target_id
        return GoalResponse.ACCEPT
    
    async def execute_rescue_callback(self, goal_handle):
        """
        Executes a rescue by navigating to a given goal
        """
        req = goal_handle.request
        feedback = ExecuteRescue.Feedback()
        self.get_logger().info(f"[RobotController {self.robot}] Navigating to target {req.target_id} at {req.goal.pose.position.x:.2f}, {req.goal.pose.position.y:.2f}")
        # send navigation goal
        try:
            feedback.state = "NAVIGATING"
            outcome = await self.navigate(goal_handle, req.goal.pose.position, feedback)
            if outcome == 'canceled':
                return self.finish(goal_handle, False,'canceled')
            elif outcome == 'failed':
                return self.finish(goal_handle, False,'failed')
            self.stop()
            feedback.state = "DWELLING"
            start = self.get_clock().now()
            while self.get_clock().now() - start < req.dwell_time:
                if goal_handle.is_cancel_requested:
                    return self.finish(goal_handle, False,'canceled')
                await self.sleep(self.period)
            return self.finish(goal_handle, True,'succeeded')

        finally:
            self.stop()
            self.target_id = ''


    async def navigate(self, goal_handle, zone, feedback):
        """Returns 'arrived', 'failed' or 'canceled'."""
        if not self.nav_client.server_is_ready():
            return 'failed'
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_stamped(zone)
        nav = await self.nav_client.send_goal_async(goal)
        if not nav.accepted:
            return 'failed'
        result = nav.get_result_async()
        while not result.done():
            if goal_handle.is_cancel_requested:
                await nav.cancel_goal_async()
                return 'canceled'
            feedback.distance_remaining = float(self.distance_to(zone))
            goal_handle.publish_feedback(feedback)
            await self.sleep(self.period)
        return 'arrived' if result.result().status == GoalStatus.STATUS_SUCCEEDED else 'failed'

    def finish(self, goal_handle, success, message):
        if success:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.get_logger().info(f'{goal_handle.request.target_id}: {message}')
        result = ExecuteRescue.Result()
        result.success = success
        result.message = message
        result.finished = self.get_clock().now().to_msg()
        return result

def main():
    rclpy.init()
    try:
        rclpy.spin(RobotController())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()