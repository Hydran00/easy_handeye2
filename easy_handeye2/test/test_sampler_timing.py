from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
import tf2_ros

from easy_handeye2.handeye_sampler import HandeyeSampler


def transform(seconds):
    result = TransformStamped()
    result.header.stamp = Time(seconds=seconds).to_msg()
    return result


def sampler(robot_time, camera_time, calibration_type='eye_on_base'):
    result = HandeyeSampler.__new__(HandeyeSampler)
    result.handeye_parameters = SimpleNamespace(
        calibration_type=calibration_type, robot_base_frame='base',
        robot_effector_frame='hand', tracking_base_frame='camera',
        tracking_marker_frame='board')
    result.max_sample_age = 1.0
    result.node = Mock()
    result.node.get_clock().now.return_value = Time(seconds=10)
    result.tfBuffer = Mock()
    result.tfBuffer.lookup_transform.side_effect = [
        transform(robot_time), transform(camera_time), transform(9.5), transform(9.5)]
    return result


@pytest.mark.parametrize('mode,robot_frames', [
    ('eye_on_base', ('hand', 'base')),
    ('eye_in_hand', ('base', 'hand')),
])
def test_samples_same_available_instant_without_waiting(mode, robot_frames):
    subject = sampler(9.9, 9.5, mode)
    assert subject._get_transforms() is not None
    calls = subject.tfBuffer.lookup_transform.call_args_list
    assert calls[2].args[:2] == robot_frames
    assert calls[3].args[:2] == ('camera', 'board')
    for call in calls[2:]:
        assert call.args[2].nanoseconds == 9_500_000_000
        assert call.args[3].nanoseconds == 0


@pytest.mark.parametrize('robot_time,camera_time', [(1, 9.9), (9.9, 1), (11, 11)])
def test_rejects_stale_or_future_data(robot_time, camera_time):
    subject = sampler(robot_time, camera_time)
    assert subject._get_transforms() is None
    assert subject.tfBuffer.lookup_transform.call_count == 2


def test_missing_history_is_rejected():
    subject = sampler(9.9, 9.5)
    subject.tfBuffer.lookup_transform.side_effect = [
        transform(9.9), transform(9.5), tf2_ros.ExtrapolationException('no overlap')]
    assert subject._get_transforms() is None


def test_static_chain_does_not_force_time_zero():
    subject = sampler(0, 9.5)
    assert subject._get_transforms() is not None
    assert subject.tfBuffer.lookup_transform.call_args_list[2].args[2].nanoseconds == 9_500_000_000
