#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, PoseStamped
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath import UnitQuaternion
from spatialmath.base import tr2eul
import time
class MotionPlanner(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        
        # Declare parameters
        super().__init__("linear_trajectory_publisher")

        self.declare_parameter("camera_to_calibrate", "cam1")
        self.declare_parameter("robot_base_frame", "lbr_link_0")
        self.declare_parameter("robot_effector_frame", "lbr_link_ee")
        self.declare_parameter("topic_name", "/lbr/target_frame")
        self.declare_parameter("waiting_time", 3.0)

        self.camera_to_calibrate = self.get_parameter("camera_to_calibrate").get_parameter_value().string_value
        self.base = self.get_parameter("robot_base_frame").get_parameter_value().string_value
        self.end_effector = self.get_parameter("robot_effector_frame").get_parameter_value().string_value
        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        
        # Initialize the publisher for PoseStamped messages
        self.publisher_ = self.create_publisher(PoseStamped, self.topic_name, 10)

        # Initialize tf2 for transforming coordinates
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_freq = 1000  # 1 kHz
        self.waiting_time = self.get_parameter("waiting_time").get_parameter_value().double_value
        # Position increment

        self.starting_position = np.array([0.5818, -0.1667, 0.5312, -0.05, 0.998, -0.003, 0.0357])
        # Normalize quaternion part
        self.starting_position[3:7] = self.starting_position[3:7] / np.linalg.norm(self.starting_position[3:7])

        if self.camera_to_calibrate == "cam1":
            self.target_keypoints = np.array([
                [0.53, 0.11, 0.51,  -0.25, 0.932, -0.217, -0.14],
                [0.72, -0.20, 0.474, -0.51, 0.85, 0.07, -0.03],
                [0.36, -0.15, 0.52, -0.29, 0.92, -0.06,0.24],
                [0.31, -0.48, 0.41,  0.147, 0.95, 0.19, 0.2],
                [0.44, -0.03, 0.53, -0.54, 0.82, -0.10, 0.17],
                # starting position
                self.starting_position.tolist(),
                # [0.0, 0.0, 0.0,  0.0,0.0,0.0,1.0],
            ])
        elif self.camera_to_calibrate == "cam2":
            self.target_keypoints = np.array([
                # self.starting_position.tolist(),
                [0.55454, -0.16556, 0.53259, 0.33266, 0.85817, -0.38668, 0.057999],
                [0.52717, -0.16629, 0.53597, 0.40037, 0.78332, 0.30039, 0.36862],
                [0.58247, -0.16705, 0.53331, 0.91821, 0.38372, -0.095565, -0.022822],
                [0.57753, -0.162, 0.5375, 0.91809, 0.38502, -0.088328, -0.032708],
                [0.55005, -0.22137, 0.4009, 0.60624, 0.78302, 0.083376, -0.1114],
                [0.55005, -0.22143, 0.401, 0.12746, 0.98431, -0.11439, -0.0424],
                [0.6078, -0.14283, 0.40282, 0.16126, 0.93077, -0.061053, 0.32238],
                self.starting_position.tolist()
            ])

        else:
            self.get_logger().error("Unknown camera to calibrate, exiting...")
            exit(1)

        # normalize quaternion part
        self.target_keypoints[:, 3:7] = self.target_keypoints[:, 3:7] / np.linalg.norm(self.target_keypoints[:, 3:7], axis=1, keepdims=True)

        # self.step_size = np.array([
        #     [-0.00, -0.000, -0.0],
        #     [-0.04, 0.000, -0.0],
        #     [-0.00, 0.003, -0.0],
        #     [-0.00, 0.000, 0.1],
        #     [-0.00, 0.000, 0.0],
        #     [-0.00, 0.000, 0.0],
        #     [-0.00, 0.000, 0.0],
        #     [-0.05, 0.000, 0.0],
        # ])
        # # Orientation increment in RPY
        # self.step_orientation = np.array([
        #     [np.pi/14, 0.0, 0.0],
        #     [-np.pi/10, np.pi/10, 0.0],
        #     [0, -np.pi/14, np.pi/10],
        #     [0, 0, -np.pi/10],
        #     [0, 0, np.pi/10],
        #     [np.pi/10, 0, 0.0],
        #     [-np.pi/8, -0, np.pi/8],
        #     [0, 0, -np.pi/8],
        # ])

        # if self.step_size.shape[0] != self.step_orientation.shape[0]:
        #     raise ValueError("step_size and step_orientation must have the same number of rows")
        #     exit(1)

        self.initial_orientation = None
        self.initial_position = None
        self.velocity = 0.03  # Speed of the linear trajectory
        self.ang_velocity = 0.1  # Speed of the angular trajectory
        self.traj_idx = 0
        self.timer = self.create_timer(1.0 / self.pub_freq, self.publish_trajectory)


    def get_current_transform(self):
        try:
            transform_msg: TransformStamped = self.tf_buffer.lookup_transform(
                self.base, self.end_effector, rclpy.time.Time()
            )
            return transform_msg
        except Exception as e:
            self.get_logger().warn(f"Failed to get transform: {e}")
            return None
    
    def compute_ctraj(self):
        # Create a SE3 object from the quaternion and translation vector
        # traj list is Nx7
        traj_list = np.empty((0, 7))
                    # add waiting time
        for i in range(int(self.waiting_time * self.pub_freq)):
            traj_list = np.vstack((traj_list, np.zeros((1, 7))))
            traj_list[i, 0:3] = self.initial_position
            traj_list[i, 3:7] = self.initial_orientation


        for i in range(len(self.target_keypoints)):
            # lin_distance_between_waypoints = self.velocity / self.pub_freq
            # number_of_samples_lin = int(
            #     np.linalg.norm(self.step_size[i]) / lin_distance_between_waypoints
            # )
            # ang_distance_between_waypoints = self.ang_velocity / self.pub_freq
            # number_of_samples_ang = int(
            #     np.linalg.norm(self.step_orientation[i]) / ang_distance_between_waypoints
            # )
            number_of_samples = 6000 #max(number_of_samples_lin, number_of_samples_ang)
            self.get_logger().info("Number of samples: " + str(number_of_samples))
            if i==0:
                T_start = SE3.Rt(
                    UnitQuaternion(
                        s=self.initial_orientation[3], v=self.initial_orientation[:3], norm=True
                    ).R,
                    self.initial_position,
                )
            else:
                # use last_point as the new start point
                T_start = traj_list[-1]
                T_start = SE3.Rt(
                    UnitQuaternion(
                        s=T_start[6], v=T_start[3:6], norm=True
                    ).R,
                    T_start[:3],
                )
            # self.get_logger().info("Start position: " + str(T_start.t) + str(self.step_size[i]))
            # pos_end = self.step_size[i] + T_start.t
            # orient_end_RPY = self.step_orientation[i] + R.from_matrix(T_start.R).as_euler("xyz")
            # orient_end = R.from_euler("xyz", orient_end_RPY).as_quat()
            pose_end = self.target_keypoints[i]
            pos_end = pose_end[:3]
            orient_end = pose_end[3:7]

            T_end = SE3.Rt(
                UnitQuaternion(
                    s=orient_end[3], v=orient_end[:3], norm=True
                ).R,
                pos_end,
            )
            self.get_logger().info("Start position: " + str(T_start.t))
            self.get_logger().info("End position: " + str(T_end.t))


            tg = rtb.ctraj(T_start, T_end, t=number_of_samples)
            print("Computed trajectory, len is ", len(tg))

            traj = np.zeros((len(tg) + int(self.waiting_time * self.pub_freq), 7))
            # Convert to numpy xyz qx qy qz qw
            for j in range(len(tg)):
                traj[j, 0:3] = tg[j].t
                traj[j, 3:7] = R.from_matrix(tg[j].R).as_quat()
            
            # add waiting time
            for j in range(int(self.waiting_time * self.pub_freq)):
                traj[j + len(tg)] = traj[len(tg) - 1]
            
            traj_list = np.vstack((traj_list, traj))

            print("Trajectory shape: ", traj_list.shape)


        self.trajectory = traj_list
        print("Trajectory ", self.trajectory)
        time.sleep(1.0)


    
    
    def publish_trajectory(self):
        transform_msg = self.get_current_transform()
        if transform_msg is None:
            return

        if self.initial_orientation is None:
            self.initial_orientation = np.array(
                [
                    transform_msg.transform.rotation.x,
                    transform_msg.transform.rotation.y,
                    transform_msg.transform.rotation.z,
                    transform_msg.transform.rotation.w,
                ]
            )
            self.initial_position = np.array(
                [
                    transform_msg.transform.translation.x,
                    transform_msg.transform.translation.y,
                    transform_msg.transform.translation.z,
                ]
            )
            if np.linalg.norm(self.initial_position - [0.5818,-0.166, 0.531] ) > 0.005:
                self.get_logger().warn("Initial position is not correct, exiting...")
                exit(1)
            else:
                self.get_logger().info("Initial position is correct, continuing...")
            self.commanded_x = self.initial_position[0]
            self.commanded_y = self.initial_position[1]
            self.commanded_z = self.initial_position[2]
            self.commanded_qx = self.initial_orientation[0]
            self.commanded_qy = self.initial_orientation[1]
            self.commanded_qz = self.initial_orientation[2]
            self.commanded_qw = self.initial_orientation[3]

            self.compute_ctraj()
            self.get_logger().info("Publishing trajectory...")


        self.current_pose = (
            transform_msg.transform.translation.x,
            transform_msg.transform.translation.y,
            transform_msg.transform.translation.z,
            transform_msg.transform.rotation.x,
            transform_msg.transform.rotation.y,
            transform_msg.transform.rotation.z,
            transform_msg.transform.rotation.w,
        )

        if self.traj_idx >= len(self.trajectory):
            self.get_logger().info("Trajectory completed.")
            exit(0)
        traj_point = self.trajectory[self.traj_idx]
        self.traj_idx += 1

        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = self.base
        target_pose.pose.position.x = traj_point[0]
        target_pose.pose.position.y = traj_point[1]
        target_pose.pose.position.z = traj_point[2]
        target_pose.pose.orientation.x = traj_point[3]
        target_pose.pose.orientation.y = traj_point[4]
        target_pose.pose.orientation.z = traj_point[5]
        target_pose.pose.orientation.w = traj_point[6]

        # Publish the message
        # self.get_logger().info("Publishing target pose: " + str(target_pose))
        self.publisher_.publish(target_pose)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
