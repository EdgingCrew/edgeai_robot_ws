#!/usr/bin/env python3
import numpy as np
import cv2
import math
import json
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

def quat_to_matrix(q):
    """Rotation matrix (3x3) from a quaternion message (x, y, z, w)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)]])

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.15)   # marker side in meters
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('optical_frame', 'camera_optical_frame')
        self.declare_parameter('max_distance', 3.0)   # ignore observations farther than this (m)

        image_topic = self.get_parameter('image_topic').value
        dict_name = self.get_parameter('dictionary').value

        # Build the detector once; it is reused for every image.
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self.detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

        self.sub = self.create_subscription(
            Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'Listening on {image_topic}, dictionary {dict_name}')

        # Marker corners in the marker's own frame (z = 0 plane), in the same
        # order as cv2.aruco returns them: top-left, top-right, bottom-right, bottom-left.
        s = self.get_parameter('marker_size').value / 2.0
        self.obj_points = np.array(
            [[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)

        # Camera intrinsics are filled in by the first CameraInfo message.
        self.K = None
        self.dist = None
        self.info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.on_info, qos_profile_sensor_data)
        self.target_frame = self.get_parameter('target_frame').value
        self.optical_frame = self.get_parameter('optical_frame').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)   # fills the buffer from /tf
        self.max_distance = self.get_parameter('max_distance').value
        self.targets = {}   # marker id -> running sums, see update_target()
        self.report_timer = self.create_timer(1.0, self.print_targets)
        self.declare_parameter('output_file', 'aruco_targets.json') # save in current folder
        self.marker_pub = self.create_publisher(MarkerArray, '/aruco_targets', 10)

    def on_info(self, msg):
        if self.K is not None:
            return
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
        self.get_logger().info(f'Got camera intrinsics: fx={self.K[0, 0]:.1f}')

    def update_target(self, marker_id, x, y, yaw):
        t = self.targets.setdefault(
            marker_id, {'n': 0, 'sx': 0.0, 'sy': 0.0, 'sc': 0.0, 'ss': 0.0})
        t['n'] += 1
        t['sx'] += x
        t['sy'] += y
        # Angles cannot be averaged directly (+179 and -179 would give 0),
        # so sum the unit vectors and take the angle of the sum.
        t['sc'] += math.cos(yaw)
        t['ss'] += math.sin(yaw)

    def current_targets(self):
        """Merged targets as a list of dicts, one per marker id."""
        result = []
        for marker_id, t in sorted(self.targets.items()):
            n = t['n']
            result.append({
                'id': marker_id,
                'type': 'unknown',   # TBD: the rulebook defines the marker types
                'x': t['sx'] / n,
                'y': t['sy'] / n,
                'yaw': math.atan2(t['ss'], t['sc']),   # radians
                'n': n})
        return result

    def print_targets(self):
        targets = self.current_targets()
        for tg in targets:
            self.get_logger().info(
                f'TARGET id={tg["id"]} x={tg["x"]:+.2f} y={tg["y"]:+.2f} '
                f'yaw={math.degrees(tg["yaw"]):+.0f} deg (n={tg["n"]})')
        self.publish_markers(targets)

    def publish_markers(self, targets):
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for tg in targets:
            # Arrow: starts at the marker and points the way the marker faces.
            arrow = Marker()
            arrow.header.frame_id = self.target_frame
            arrow.header.stamp = stamp
            arrow.ns = 'aruco_arrow'
            arrow.id = tg['id']
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = tg['x']
            arrow.pose.position.y = tg['y']
            arrow.pose.position.z = 0.11
            arrow.pose.orientation.z = math.sin(tg['yaw'] / 2.0)   # yaw -> quaternion
            arrow.pose.orientation.w = math.cos(tg['yaw'] / 2.0)
            arrow.scale.x = 0.3     # arrow length (m)
            arrow.scale.y = 0.03
            arrow.scale.z = 0.03
            arrow.color.g = 1.0
            arrow.color.a = 1.0
            array.markers.append(arrow)

            # Text label above the marker.
            label = Marker()
            label.header.frame_id = self.target_frame
            label.header.stamp = stamp
            label.ns = 'aruco_text'
            label.id = tg['id']
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = tg['x']
            label.pose.position.y = tg['y']
            label.pose.position.z = 0.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12    # text height (m)
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'ID {tg["id"]}'
            array.markers.append(label)
        self.marker_pub.publish(array)

    def save_targets(self):
        path = os.path.expanduser(self.get_parameter('output_file').value)
        with open(path, 'w') as f:
            json.dump(self.current_targets(), f, indent=2)
        self.get_logger().info(f'Saved {len(self.targets)} target(s) to {path}')

    def on_image(self, msg):
        # The image arrives as a flat byte array (rgb8): reshape to rows x cols x 3.
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

        corners, ids, _rejected = self.detector.detectMarkers(gray)
        if ids is None:
            return

        if self.K is None:
            return # wait for camera_info

        for marker_corners, marker_id in zip(corners, ids.flatten()):
            img_points = marker_corners.reshape(4, 2).astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(
                self.obj_points, img_points, self.K, self.dist,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            p_cam = tvec.flatten()

            if np.linalg.norm(p_cam) > self.max_distance:
                continue

            try:
                t = self.tf_buffer.lookup_transform(
                    self.target_frame, self.optical_frame, Time.from_msg(msg.header.stamp))
            except TransformException as ex:
                self.get_logger().warn(f'No transform yet: {ex}', throttle_duration_sec=2.0)
                return

            R = quat_to_matrix(t.transform.rotation)
            tr = t.transform.translation
            p_map = R @ p_cam + np.array([tr.x, tr.y, tr.z])

            # Marker normal = the marker's z axis; it points out of the marker face.
            R_cam_marker, _ = cv2.Rodrigues(rvec)
            normal_map = R @ R_cam_marker[:, 2]
            yaw = math.atan2(normal_map[1], normal_map[0])

            self.update_target(int(marker_id), p_map[0], p_map[1], yaw)

            self.get_logger().info(
                f'id={marker_id} dist={np.linalg.norm(p_cam):.2f} m',
                throttle_duration_sec=1.0)

def main():
    rclpy.init()
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_targets()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
