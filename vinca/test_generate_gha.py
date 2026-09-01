"""Tests for the GitHub Actions pipeline generation."""

import pytest

from vinca import config
from vinca.generate_gha import get_stage_name


@pytest.fixture(autouse=True)
def rolling_distro():
    previous = config.ros_distro
    config.ros_distro = "rolling"
    yield
    config.ros_distro = previous


@pytest.mark.parametrize(
    "package,expected",
    [
        ("ros-rolling-rclcpp", "rclcpp"),
        ("ros-rolling-ament-package", "ament-package"),
        ("ros2-rclcpp", "ros2-rclcpp"),
        ("ros2-ament-package", "ros2-ament-package"),
        ("ros2-distro-mutex", "ros2-distro-mutex"),
        ("ros-humble-rclcpp", "ros-humble-rclcpp"),
    ],
)
def test_get_stage_name_strips_only_the_legacy_prefix(package, expected):
    assert get_stage_name([package]) == expected


def test_get_stage_name_joins_a_batch():
    batch = ["ros-rolling-rclcpp", "ros2-ament-package"]
    assert get_stage_name(batch) == "rclcpp ros2-ament-package"
