#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Node: ArUco marker detection + TF broadcasting
Author: <your_name>
Description:
  Detects ArUco markers from camera feed, estimates their poses, and publishes TF transforms.
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from scipy.spatial.transform import Rotation as R
import tf2_ros

class ArucoTFSender(Node):
    def __init__(self):
        super().__init__('aruco_tf_publisher')

        # ---- ROS setup ----
        self.bridge = CvBridge()
        self.br = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera_info', self.camerainfo_callback, 10)
        self.create_subscription(String, '/ur5/request', self.request_callback, 10)

        # ---- Variables ----
        self.camera_matrix = None
        self.dist_coeffs = None
        self.detect_cart = False

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_size = 0.06  # meters

        self.get_logger().info("Aruco TF Publisher initialized")

    # ------------------- Callbacks -------------------
    def camerainfo_callback(self, msg: CameraInfo):
        """Dynamically receive camera calibration matrix"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("Camera parameters received")

    def request_callback(self, msg: String):
        if msg.data.strip().upper() == "REQUEST_FERTILIZER":
            self.detect_cart = True
            self.get_logger().info("Cart detection enabled via request")

    def image_callback(self, msg: Image):
        """Main image processing callback"""
        if self.camera_matrix is None:
            self.get_logger().warn( "Waiting for CameraInfo...")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or len(ids) == 0:
            cv2.imshow("Aruco Detection", frame)
            cv2.waitKey(1)
            return

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == 6 and not self.detect_cart:
                continue  # cart disabled until requested

            rvec = rvecs[i]
            tvec = tvecs[i]
            self.publish_tf(marker_id, rvec, tvec)

            # visualization
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
            c = np.mean(corners[i][0], axis=0).astype(int)
            cv2.circle(frame, tuple(c), 5, (0, 0, 255), -1)

        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)

    # ------------------- Core function -------------------
    def publish_tf(self, marker_id, rvec, tvec):
        """Compute and send TF transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera"
        t.child_frame_id = f"aruco_{marker_id}"

        # Convert to meters
        t.transform.translation.x = float(tvec[0][0])
        t.transform.translation.y = float(tvec[0][1])
        t.transform.translation.z = float(tvec[0][2])

        # Convert rotation vector to quaternion (OpenCV to ROS order)
        r = R.from_rotvec(rvec[0])
        q = r.as_quat()  # x, y, z, w
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Optional: flip or rotate to match convention (if required)
        # For example, to rotate 180° around X
        # flip_r = R.from_euler('x', np.pi)
        # q_flipped = (flip_r * r).as_quat()

        self.br.sendTransform(t)
        self.get_logger().debug(f"Published TF for ArUco ID {marker_id}")

# ------------------- Main -------------------

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTFSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


