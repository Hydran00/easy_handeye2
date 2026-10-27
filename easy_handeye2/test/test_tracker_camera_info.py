from unittest.mock import Mock

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from easy_handeye2.chessboard_tracker import ChessboardTracker


def camera_info():
    msg = CameraInfo()
    msg.width, msg.height = 640, 480
    msg.header.frame_id = 'optical'
    msg.distortion_model = 'rational_polynomial'
    msg.k = [500., 0., 320., 0., 501., 240., 0., 0., 1.]
    msg.d = [0.1, -0.2, 0.001, 0.002, 0.3, 0.4, 0.5, 0.6]
    return msg


def subject():
    node = Mock()
    node.camera_optical_frame = 'optical'
    node.camera_info_topic = '/k4a/rgb/camera_info'
    node.board_type = 'charuco'
    node.camera_matrix = np.eye(3)
    node.dist_coeffs = np.zeros((1, 8))
    return node


def test_updates_pnp_and_charuco_intrinsics_from_topic():
    node = subject()
    msg = camera_info()
    ChessboardTracker.camera_info_callback(node, msg)
    np.testing.assert_array_equal(node.camera_matrix, np.array(msg.k).reshape(3, 3))
    np.testing.assert_array_equal(node.dist_coeffs.ravel(), msg.d)
    params = node.charuco_detector.setCharucoParameters.call_args.args[0]
    np.testing.assert_array_equal(params.cameraMatrix, node.camera_matrix)
    np.testing.assert_array_equal(params.distCoeffs, node.dist_coeffs)
    assert node.camera_info is msg


def test_rejects_wrong_frame_uncalibrated_and_unsupported_model():
    for field, value in [('distortion_model', 'equidistant'), ('k', [0.] * 9)]:
        node = subject()
        msg = camera_info()
        setattr(msg, field, value)
        ChessboardTracker.camera_info_callback(node, msg)
        assert node.camera_info is None
    node = subject()
    msg = camera_info()
    msg.header.frame_id = 'depth'
    ChessboardTracker.camera_info_callback(node, msg)
    assert node.camera_info is None


def test_skips_images_without_matching_info():
    for info in [None, camera_info()]:
        node = subject()
        node.camera_info = info
        ChessboardTracker.image_callback(node, Image())
        node.bridge.imgmsg_to_cv2.assert_not_called()
        node.tf_broadcaster.sendTransform.assert_not_called()
