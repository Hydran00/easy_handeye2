from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node as RclpyNode, QoSProfile
from sensor_msgs.msg import CameraInfo


def get_camera_intrinsics(camera_info_topic: str, timeout_sec: float = 10.0) -> dict:
    """
    Spin a temporary ROS2 node to read one CameraInfo message
    and return the intrinsic parameters as a dict.
    """
    rclpy.init()

    class CameraInfoReader(RclpyNode):
        def __init__(self):
            super().__init__('camera_info_reader_tmp')
            self.intrinsics = None
            self.sub = self.create_subscription(
                CameraInfo,
                camera_info_topic,
                self.callback,
                # use qos transient local
                QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
            )
            print("Listening on " + camera_info_topic)

        def callback(self, msg: CameraInfo):
            self.intrinsics = {
                "fx": float(msg.k[0]),
                "fy": float(msg.k[4]),
                "cx": float(msg.k[2]),
                "cy": float(msg.k[5]),
                "k1": float(msg.d[0]) if len(msg.d) > 0 else 0.0,
                "k2": float(msg.d[1]) if len(msg.d) > 1 else 0.0,
                "p1": float(msg.d[2]) if len(msg.d) > 2 else 0.0,
                "p2": float(msg.d[3]) if len(msg.d) > 3 else 0.0,
                "k3": float(msg.d[4]) if len(msg.d) > 4 else 0.0,
            }
    node = CameraInfoReader()

    import time
    deadline = time.time() + timeout_sec
    while rclpy.ok() and node.intrinsics is None:
        print("Waiting")
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() > deadline:
            node.destroy_node()
            rclpy.shutdown()
            raise RuntimeError(f"Timeout: no CameraInfo received on '{camera_info_topic}' within {timeout_sec}s")

    intrinsics = node.intrinsics
    node.destroy_node()
    rclpy.shutdown()
    return intrinsics


def launch_setup(context, *args, **kwargs):
    # Read intrinsics at launch time from the CameraInfo topic
    intrinsics = get_camera_intrinsics('/cam2_32449015/camera_info')
    print(f"[launch] Camera intrinsics read from /cam2_32449015/camera_info: {intrinsics}")

    calib_node = Node(
        package='easy_handeye2',
        executable='aruco_tracker',
        name='aruco_tracker',
        parameters=[{
            "image_topic": "/camera2/image",
            "marker_id": 2,
            "marker_length": 0.13,
            "camera_frame": "cam2_32449015_image",
            "marker_frame": "aruco_marker_frame",
            "fx": intrinsics["fx"],
            "fy": intrinsics["fy"],
            "cx": intrinsics["cx"],
            "cy": intrinsics["cy"],
            "k1": intrinsics["k1"],
            "k2": intrinsics["k2"],
            "p1": intrinsics["p1"],
            "p2": intrinsics["p2"],
            "k3": intrinsics["k3"],
        }]
    )

    handeye_server = Node(
        package='easy_handeye2',
        executable='handeye_server',
        name='handeye_server',
        parameters=[{
            'name': "calibrator_cam_1",
            'calibration_type': "eye_on_base",
            'tracking_base_frame': "cam2_32449015_image",
            'tracking_marker_frame': "aruco_marker_frame",
            'robot_base_frame': "lbr_link_0",
            'robot_effector_frame': "lbr_link_ee"
        }]
    )

    handeye_rqt_calibrator = Node(
        package='easy_handeye2',
        executable='rqt_calibrator.py',
        name='handeye_rqt_calibrator',
        parameters=[{
            'name': "calibrator_cam_1",
            'calibration_type': "eye_on_base",
            'tracking_base_frame': "cam2_32449015_image",
            'tracking_marker_frame': "aruco_marker_frame",
            'robot_base_frame': "lbr_link_0",
            'robot_effector_frame': "lbr_link_ee"
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', get_package_share_directory('easy_handeye2') + '/rviz/rviz_calib.rviz'],
    )

    return [
        calib_node,
        handeye_server,
        handeye_rqt_calibrator,
        rviz_node,
    ]


def generate_launch_description():
    print("Calibrating camera 1: SN 32449015")

    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])