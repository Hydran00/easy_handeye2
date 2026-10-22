"""Calibrate Polaris -> Franka base with an NDI marker rigidly on the EE.

Start the Franka bringup first: its robot_state_publisher must publish the
base -> EE chain from live joint states. No camera or probe-tip TF is needed.
Polaris and Franka bases must remain fixed throughout acquisition.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("name", default_value="calibrator_holder_ifl"),
        DeclareLaunchArgument(
            "robot_base_frame",
            default_value="panda_link0",
            description="Franka base frame in the live TF tree",
        ),
        DeclareLaunchArgument(
            "robot_effector_frame",
            default_value="panda_link8",
            description="Franka EE frame rigidly attached to the NDI marker",
        ),
        DeclareLaunchArgument(
            "polaris_frame",
            default_value="polaris_frame",
            description="Reference frame of the NDI pose messages",
        ),
        DeclareLaunchArgument("marker_frame", default_value="marker"),
        DeclareLaunchArgument("rigid_bodies_topic", default_value="/rigid_poses"),
        DeclareLaunchArgument(
            "ndi_sensor_name",
            default_value="marker",
            description="Rigid body name in RigidArray.frames, not a TF frame",
        ),
        DeclareLaunchArgument(
            "start_marker_tf",
            default_value="true",
            description="Set false if Polaris already publishes the marker TF",
        ),
        DeclareLaunchArgument(
            "max_sample_age",
            default_value="1.0",
            description="Maximum age of synchronized TF samples in seconds",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
    ]

    # Exchange the roles of the two measuring systems so eye_on_base returns
    # polaris_T_franka_base directly, rather than its inverse:
    # polaris_T_marker = polaris_T_base * base_T_ee * ee_T_marker.
    # The sampler reads the live Franka base_T_ee via TF (joint-state FK).
    params = {
        "name": LaunchConfiguration("name"),
        "calibration_type": "eye_on_base",
        "robot_base_frame": LaunchConfiguration("polaris_frame"),
        "robot_effector_frame": LaunchConfiguration("marker_frame"),
        "tracking_base_frame": LaunchConfiguration("robot_base_frame"),
        "tracking_marker_frame": LaunchConfiguration("robot_effector_frame"),
    }

    return LaunchDescription(
        args
        + [
            Node(
                package="polaris_bringup",
                executable="rigid_poses_tf.py",
                name="holder_rigid_poses_tf",
                condition=IfCondition(LaunchConfiguration("start_marker_tf")),
                remappings=[("rigid_poses", LaunchConfiguration("rigid_bodies_topic"))],
                parameters=[
                    {
                        "sensor_name": LaunchConfiguration("ndi_sensor_name"),
                        "marker_frame": LaunchConfiguration("marker_frame"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="easy_handeye2",
                executable="handeye_server",
                name="handeye_server",
                parameters=[
                    params,
                    {
                        "max_sample_age": ParameterValue(
                            LaunchConfiguration("max_sample_age"), value_type=float
                        )
                    },
                ],
                output="screen",
            ),
            Node(
                package="easy_handeye2",
                executable="rqt_calibrator.py",
                name="handeye_rqt_calibrator",
                parameters=[params],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    get_package_share_directory("easy_handeye2")
                    + "/rviz/rviz_calib.rviz",
                    "-f",
                    LaunchConfiguration("polaris_frame"),
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
