from unittest.mock import Mock

import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation

from easy_handeye2.chessboard_tracker import ChessboardTracker


def tracker(link='camera_link', optical='optical'):
    # Exercise publishing without starting a ROS node or requiring a camera.
    subject = Mock()
    subject.camera_link = link
    subject.camera_optical_frame = optical
    subject.marker_frame = 'board'
    return subject


def test_composes_rotation_and_translation_in_link_frame():
    subject = tracker()
    extrinsic = TransformStamped()
    extrinsic.transform.translation.x = 0.1
    extrinsic.transform.translation.y = -0.2
    link_rotation = Rotation.from_euler('z', 90, degrees=True)
    q = link_rotation.as_quat()
    r = extrinsic.transform.rotation
    r.x, r.y, r.z, r.w = q
    subject.tf_buffer.lookup_transform.return_value = extrinsic
    marker_rotation = Rotation.from_euler('x', 30, degrees=True)
    stamp = TransformStamped().header.stamp
    stamp.sec = 42
    assert ChessboardTracker.publish_transform(
        subject, marker_rotation.as_rotvec(), np.array([[1.], [2.], [3.]]), stamp)
    published = subject.tf_broadcaster.sendTransform.call_args.args[0]
    assert published.header.frame_id == 'camera_link'
    assert published.child_frame_id == 'board'
    assert published.header.stamp.sec == 42
    p = published.transform.translation
    np.testing.assert_allclose([p.x, p.y, p.z], [-1.9, .8, 3.])
    q = published.transform.rotation
    np.testing.assert_allclose(
        Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
        (link_rotation * marker_rotation).as_matrix(), atol=1e-12)
    assert subject.tf_buffer.lookup_transform.call_args.args[:2] == ('camera_link', 'optical')


def test_missing_extrinsic_does_not_publish_mislabeled_pose():
    subject = tracker()
    subject.tf_buffer.lookup_transform.side_effect = tf2_ros.LookupException('missing')
    assert not ChessboardTracker.publish_transform(
        subject, np.zeros(3), np.ones(3), TransformStamped().header.stamp)
    subject.tf_broadcaster.sendTransform.assert_not_called()


def test_same_camera_frame_needs_no_lookup():
    subject = tracker('optical', 'optical')
    assert ChessboardTracker.publish_transform(
        subject, np.zeros(3), np.ones(3), TransformStamped().header.stamp)
    subject.tf_buffer.lookup_transform.assert_not_called()
