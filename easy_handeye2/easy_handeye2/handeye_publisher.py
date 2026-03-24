#!/usr/bin/env python

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import ParameterType, ParameterDescriptor
import tf2_ros
import geometry_msgs.msg
from easy_handeye2.handeye_calibration import load_calibration
from scipy.spatial.transform import Rotation as R
import numpy as np
class HandeyePublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__('handeye_publisher')

        self.declare_parameter('name', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        name = self.get_parameter('name').get_parameter_value().string_value

        self.get_logger().info(f'Loading the calibration with name {name}')

        self.calibration = load_calibration(name)
        parameters = self.calibration.parameters

        if parameters.calibration_type == 'eye_in_hand':
            orig = parameters.robot_effector_frame
        else:
            orig = parameters.robot_base_frame
        dest = parameters.tracking_base_frame

        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.static_transformStamped = geometry_msgs.msg.TransformStamped()

        self.static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        # self.static_transformStamped.header.frame_id = dest #orig
        # self.static_transformStamped.child_frame_id = orig #dest

        self.static_transformStamped.header.frame_id = parameters.tracking_base_frame
        self.static_transformStamped.child_frame_id = parameters.robot_base_frame
        self.get_logger().info(f"Publishing static transform from {parameters.tracking_base_frame} to {parameters.robot_base_frame}")
        # invert transform
        
        self.static_transformStamped.transform = self.calibration.transform
        
        transform = self.static_transformStamped.transform
        
        matrix4x4 = np.eye(4)
        matrix4x4[:3, :3] = R.from_quat([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]).as_matrix()
        matrix4x4[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]

        inv_matrix4x4 = np.linalg.inv(matrix4x4)
        inv_rotation = R.from_matrix(inv_matrix4x4[:3, :3]).as_quat()
        inv_translation = inv_matrix4x4[:3, 3]
        self.static_transformStamped.transform.translation.x = inv_translation[0]
        self.static_transformStamped.transform.translation.y = inv_translation[1]
        self.static_transformStamped.transform.translation.z = inv_translation[2]
        self.static_transformStamped.transform.rotation.x = inv_rotation[0]
        self.static_transformStamped.transform.rotation.y = inv_rotation[1]
        self.static_transformStamped.transform.rotation.z = inv_rotation[2]
        self.static_transformStamped.transform.rotation.w = inv_rotation[3]


        self.broadcaster.sendTransform(self.static_transformStamped)


def main(args=None):
    rclpy.init(args=args)

    handeye_publisher = HandeyePublisher()

    try:
        rclpy.spin(handeye_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        handeye_publisher.destroy_node()


if __name__ == '__main__':
    main()
