"""Publish the saved Polaris -> Franka base calibration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("name", default_value="calibrator_holder_ifl",
                              description="Name of the saved holder calibration"),
        Node(
            package="easy_handeye2",
            executable="handeye_publisher",
            name="holder_handeye_publisher",
            parameters=[{"name": LaunchConfiguration("name")}],
            output="screen",
        ),
    ])
