#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from scipy.spatial.transform import Rotation as R
class CharucoTracker(Node):
    def __init__(self):
        super().__init__('charuco_tracker')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('charuco_square_length', 0.04)  # meters
        self.declare_parameter('charuco_marker_length', 0.02)  # meters
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('marker_frame', 'charuco_board')
        # calibration params
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('k1', 0.0)
        self.declare_parameter('k2', 0.0)
        self.declare_parameter('p1', 0.0)
        self.declare_parameter('p2', 0.0)
        self.declare_parameter('k3', 0.0)

        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.charuco_square_length = self.get_parameter('charuco_square_length').get_parameter_value().double_value
        self.charuco_marker_length = self.get_parameter('charuco_marker_length').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.marker_frame = self.get_parameter('marker_frame').get_parameter_value().string_value

        self.fx = self.get_parameter('fx').get_parameter_value().double_value
        self.fy = self.get_parameter('fy').get_parameter_value().double_value
        self.cx = self.get_parameter('cx').get_parameter_value().double_value
        self.cy = self.get_parameter('cy').get_parameter_value().double_value
        self.k1 = self.get_parameter('k1').get_parameter_value().double_value
        self.k2 = self.get_parameter('k2').get_parameter_value().double_value
        self.p1 = self.get_parameter('p1').get_parameter_value().double_value
        self.p2 = self.get_parameter('p2').get_parameter_value().double_value
        self.k3 = self.get_parameter('k3').get_parameter_value().double_value
        self.camera_matrix = np.array([[self.fx, 0, self.cx],
                                       [0, self.fy, self.cy],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([[self.k1, self.k2, self.p1, self.p2, self.k3]], dtype=np.float32)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Define Charuco board
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board = cv2.aruco.CharucoBoard((7,4), self.charuco_square_length, self.charuco_marker_length, self.dictionary)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        if ids is not None:
            ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self.board)
            if ret > 4:
                success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, self.board, self.camera_matrix, self.dist_coeffs, None, None)

                if success:
                    # draw the board
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    cv2.imshow("Charuco Board", frame)
                    cv2.waitKey(1)
                    stamp = self.get_clock().now().to_msg()
                    self.publish_transform(rvec, tvec, stamp)

    def publish_transform(self, rvec, tvec, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = self.marker_frame
        transform.transform.translation = Vector3(x=tvec[0][0], y=tvec[1][0], z=tvec[2][0])

        rot_mat, _ = cv2.Rodrigues(rvec)
        q = self.rotation_matrix_to_quaternion(rot_mat)
        transform.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        q = R.from_matrix(R).as_quat()
        return q


def main(args=None):
    rclpy.init(args=args)
    node = CharucoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
