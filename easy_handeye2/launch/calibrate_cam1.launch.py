from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# from easy_handeye2.common_launch import arg_calibration_type, arg_tracking_base_frame, arg_tracking_marker_frame, arg_robot_base_frame, \
#     arg_robot_effector_frame


def generate_launch_description():
    # arg_name = DeclareLaunchArgument('name')

    node_dummy_calib_eih = Node(package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher',
                                arguments=f'--x 0 --y 0 --z 0.1 --qx 0 --qy 0 --qz 0 --qw 1'.split(' ') + ['--frame-id', "lbr_link_0",
                                                                                                           '--child-frame-id', "lbr_link_ee"])
    # calib_node = Node(package='easy_handeye2', executable='charuco_tracker', name='charuco_tracker',
    #                     parameters=[{
    #                         "image_topic": "camera_raw_1",
    #                         "charuco_square_length": 0.04347,
    #                         "charuco_marker_length": 0.03260,
    #                         "camera_frame": "cam1",
    #                         "marker_frame": "aruco_marker_frame",
    #                         "fx": 1068.55,
    #                         "fy": 1068.74,
    #                         "cx": 1122.4,
    #                         "cy": 632.579,
    #                         "k1": -0.0530216,
    #                         "k2": 0.025668,
    #                         "p1": 0.000143909,
    #                         "p2": -0.000337566,
    #                         "k3": -0.0100711
    #                     }])
    
    calib_node = Node(package='easy_handeye2', executable='aruco_tracker', name='charuco_tracker',
                        parameters=[{
                            "image_topic": "camera_raw_1",
                            "marker_id": 0,
                            "marker_length": 0.27,
                            "camera_frame": "cam1",
                            "marker_frame": "aruco_marker_frame",
                            "fx": 1046.810669,
                            "fy": 1046.810669,
                            "cx": 1138.934814,
                            "cy": 636.102234,
                            # "k1": -0.0530216,
                            # "k2": 0.025668,
                            # "p1": 0.000143909,
                            # "p2": -0.000337566,
                            # "k3": -0.0100711
                        }])

    handeye_server = Node(package='easy_handeye2', executable='handeye_server', name='handeye_server', parameters=[{
        'name': "calibrator_cam1",
        'calibration_type': "eye_in_hand",
        'tracking_base_frame': "cam1",
        'tracking_marker_frame': "aruco_marker_frame",
        'robot_base_frame': "lbr_link_0",
        'robot_effector_frame': "lbr_link_ee"
    }])

    handeye_rqt_calibrator = Node(package='easy_handeye2', executable='rqt_calibrator.py',
                                  name='handeye_rqt_calibrator',
                                  # arguments=['--ros-args', '--log-level', 'debug'],
                                  parameters=[{
                                    'name': "calibrator_cam1",
                                    'calibration_type': "eye_in_hand",
                                    'tracking_base_frame': "cam1",
                                    'tracking_marker_frame': "aruco_marker_frame",
                                    'robot_base_frame': "lbr_link_0",
                                    'robot_effector_frame': "lbr_link_ee"
                                  }])
    

    motion_planner = Node(package='easy_handeye2', executable='motion_planner', name='motion_planner', parameters=[{
        "camera_to_calibrate": "cam1",
        "robot_base_frame" : "lbr_link_0",
        "robot_effector_frame" : "lbr_link_ee",
        "topic_name" : "/lbr/target_frame",
        "waiting_time": 5.0
    }])

    return LaunchDescription([
        # arg_name,
        # arg_calibration_type,
        # arg_tracking_base_frame,
        # arg_tracking_marker_frame,
        # arg_robot_base_frame,
        # arg_robot_effector_frame,
        calib_node,
        node_dummy_calib_eih,
        # node_dummy_calib_eob,
        handeye_server,
        handeye_rqt_calibrator,
        motion_planner,
    ])
