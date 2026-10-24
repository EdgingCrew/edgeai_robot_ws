#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import os
import select
import sys
import termios
import tty
from rclpy.signals import SignalHandlerOptions

# Key -> (linear scale, angular scale). Arrow keys arrive as 3 bytes: ESC [ A/B/C/D
KEY_BINDINGS = {
    b'w': (1.0, 0.0),  b'\x1b[A': (1.0, 0.0),    # forward
    b's': (-1.0, 0.0), b'\x1b[B': (-1.0, 0.0),   # backward
    b'a': (0.0, 1.0),  b'\x1b[D': (0.0, 1.0),    # turn left
    b'd': (0.0, -1.0), b'\x1b[C': (0.0, -1.0),   # turn right
    b' ': (0.0, 0.0),  b'x': (0.0, 0.0),         # stop
}

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")

        # Parameters: the defaults can be overridden at startup
        # with --ros-arhs -p name:=value
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.15) # m/s
        self.declare_parameter("angular_speed", 0.4) # rad/s
        self.declare_parameter("publish_rate", 10.0) # Hz

        topic = self.get_parameter("cmd_vel_topic").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        rate = self.get_parameter("publish_rate").value

        self.pub = self.create_publisher(Twist, topic, 10)

        # State: the velocity that is currently being sent
        self.lin = 0.0
        self.ang = 0.0

        self.timer = self.create_timer(1.0 / rate, self.publish_cmd)
        self.get_logger().info(f"Publishing Twish on {topic} at {rate} Hz")

    def set_motion(self, linear, angular):
        """Set the wanted motion. Inputs are scale factors in [-1, 1]."""
        linear = max(-1.0, min(1.0, linear))
        angular = max(-1.0, min(1.0, angular))
        self.lin = linear * self.linear_speed
        self.ang = angular * self.angular_speed

    def publish_cmd(self):
        """Timer callback: publish the current state as a Twist message."""
        msg = Twist()
        msg.linear.x = self.lin
        msg.angular.z = self.ang
        self.pub.publish(msg)

def main(args=None):
    # NO: let Ctrl+C raise KeyboardInterrupt, so we can still send a stop in 'finally'
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleop()
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)    # remember terminal settings
    try:
        tty.setcbreak(fd)                   # read keys without pressing Enter
        print('w/a/s/d or arrow keys to drive, space to stop, q to quit')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)     # lets the timer publish
            if select.select([sys.stdin], [], [], 0)[0]:   # a key is waiting
                data = os.read(fd, 3)
                if data == b'q':
                    break
                if data in KEY_BINDINGS:
                    node.set_motion(*KEY_BINDINGS[data])
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)   # restore the terminal
        node.set_motion(0.0, 0.0)
        for _ in range(5):                  # send zero velocity a few times
            node.publish_cmd()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
