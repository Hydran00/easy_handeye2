from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

# IFL setup: Kinect camera sees a ChArUco board, the NDI Polaris tracks a rigid body.
# The Polaris plays the role of the robot: robot_base_frame = polaris_frame, robot_effector_frame = tracked marker.
#  - eye_on_base: camera is static, chessboard rigidly attached to the NDI marker -> result polaris_frame -> camera
#  - eye_in_hand: camera rigidly attached to the NDI marker, chessboard static    -> result marker -> camera

CALIBRATION_NAME = "calibrator_ifl"
CAMERA_FRAME = "camera_base"
# Kinect RGB CameraInfo uses rgb_camera_link as the image projection frame.
CAMERA_OPTICAL_FRAME = "rgb_camera_link"
CHESSBOARD_FRAME = "charuco_frame"
POLARIS_FRAME = "polaris_frame"
# kept distinct from 'probe', which polaris_bringup/vega.launch.py already uses for the probe tip
NDI_MARKER_FRAME = "marker"


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "calibration_type",
            default_value="eye_on_base",
            description="eye_on_base (static camera) or eye_in_hand (camera on the NDI marker)",
        ),
        DeclareLaunchArgument("image_topic", default_value="/k4a/rgb/image_raw"),
        DeclareLaunchArgument("camera_link", default_value=CAMERA_FRAME),
        DeclareLaunchArgument(
            "camera_optical_frame", default_value=CAMERA_OPTICAL_FRAME
        ),
        DeclareLaunchArgument(
            "rigid_bodies_topic",
            default_value="/rigid_bodies",
            description="ndi_msgs/RigidArray topic with the NDI rigid body poses",
        ),
        DeclareLaunchArgument(
            "ndi_sensor_name",
            default_value="probe",
            description="name of the rigid body in the RigidArray frames field",
        ),
        DeclareLaunchArgument(
            "board_type", default_value="charuco", description="charuco or chessboard"
        ),
        DeclareLaunchArgument(
            "square_size", default_value="0.030", description="square side [m]"
        ),
        DeclareLaunchArgument(
            "marker_size", default_value="0.022", description="ChArUco marker side [m]"
        ),
        DeclareLaunchArgument(
            "squares_x",
            default_value="5",
            description="ChArUco: number of squares along x",
        ),
        DeclareLaunchArgument(
            "squares_y",
            default_value="5",
            description="ChArUco: number of squares along y",
        ),
        DeclareLaunchArgument(
            "aruco_dictionary",
            default_value="DICT_4X4_50",
            description="ChArUco dictionary",
        ),
        DeclareLaunchArgument(
            "legacy_pattern",
            default_value="false",
            description="ChArUco boards made with OpenCV < 4.6 / calib.io and even rows",
        ),
        DeclareLaunchArgument(
            "chessboard_cols",
            default_value="5",
            description="chessboard: inner corners along x",
        ),
        DeclareLaunchArgument(
            "marker_corner_refinement",
            default_value="NONE",
            description="ChArUco marker corner refinement: NONE or SUBPIX (A/B comparison)",
        ),
        DeclareLaunchArgument(
            "chessboard_rows",
            default_value="5",
            description="chessboard: inner corners along y",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
    ]
    calibration_type = LaunchConfiguration("calibration_type")

    # /rigid_bodies (ndi_msgs/RigidArray) -> TF polaris_frame -> NDI_MARKER_FRAME, skipped when out of volume
    rigid_body_tf = Node(
        package="polaris_bringup",
        executable="rigid_poses_tf.py",
        name="handeye_rigid_poses_tf",
        remappings=[("rigid_poses", LaunchConfiguration("rigid_bodies_topic"))],
        parameters=[
            {
                "sensor_name": LaunchConfiguration("ndi_sensor_name"),
                "fallback_topic": "/rigid_poses",
                "marker_frame": NDI_MARKER_FRAME,
            }
        ],
        output="screen",
    )

    # TF resolves camera_base -> depth_camera_link -> rgb_camera_link;
    # compose with the detected RGB board pose to publish camera_base -> charuco_frame.
    chessboard_tracker = Node(
        package="easy_handeye2",
        executable="chessboard_tracker",
        name="chessboard_tracker",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_link": LaunchConfiguration("camera_link"),
                "camera_optical_frame": LaunchConfiguration("camera_optical_frame"),
                "marker_frame": CHESSBOARD_FRAME,
                "board_type": LaunchConfiguration("board_type"),
                "marker_size": ParameterValue(
                    LaunchConfiguration("marker_size"), value_type=float
                ),
                "squares_x": ParameterValue(
                    LaunchConfiguration("squares_x"), value_type=int
                ),
                "squares_y": ParameterValue(
                    LaunchConfiguration("squares_y"), value_type=int
                ),
                "aruco_dictionary": LaunchConfiguration("aruco_dictionary"),
                "marker_corner_refinement": LaunchConfiguration(
                    "marker_corner_refinement"
                ),
                "legacy_pattern": ParameterValue(
                    LaunchConfiguration("legacy_pattern"), value_type=bool
                ),
                "chessboard_cols": ParameterValue(
                    LaunchConfiguration("chessboard_cols"), value_type=int
                ),
                "chessboard_rows": ParameterValue(
                    LaunchConfiguration("chessboard_rows"), value_type=int
                ),
                "square_size": ParameterValue(
                    LaunchConfiguration("square_size"), value_type=float
                ),
                "show_preview": True,
                # Kinect RGB CameraInfo: 4096x3072, rgb_camera_link, rational_polynomial.
                # These intrinsics require images from the matching camera stream.
                "fx": 1936.1187744140625,
                "fy": 1935.5592041015625,
                "cx": 2051.88330078125,
                "cy": 1558.7591552734375,
                "k1": 0.4857649803161621,
                "k2": -2.7081496715545654,
                "p1": -0.0003263719263486564,
                "p2": -0.00037964279181323946,
                "k3": 1.6124367713928223,
                "k4": 0.3660493493080139,
                "k5": -2.531031608581543,
                "k6": 1.5364876985549927,
            }
        ],
        output="screen",
    )

    handeye_params = {
        "name": CALIBRATION_NAME,
        "calibration_type": calibration_type,
        "tracking_base_frame": LaunchConfiguration("camera_link"),
        "tracking_marker_frame": CHESSBOARD_FRAME,
        "robot_base_frame": POLARIS_FRAME,
        "robot_effector_frame": NDI_MARKER_FRAME,
    }

    handeye_server = Node(
        package="easy_handeye2",
        executable="handeye_server",
        name="handeye_server",
        parameters=[handeye_params],
        output="screen",
    )

    handeye_rqt_calibrator = Node(
        package="easy_handeye2",
        executable="rqt_calibrator.py",
        name="handeye_rqt_calibrator",
        parameters=[handeye_params],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            get_package_share_directory("easy_handeye2") + "/rviz/rviz_calib.rviz",
            "-f",
            POLARIS_FRAME,
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        args
        + [
            rigid_body_tf,
            chessboard_tracker,
            handeye_server,
            handeye_rqt_calibrator,
            rviz_node,
        ]
    )
