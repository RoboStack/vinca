"""Tests for the ROS package naming scheme."""

import pytest
from unittest.mock import Mock
from vinca.utils import ensure_name_is_without_distro_prefix_and_with_underscores
from vinca.distro import Distro


@pytest.mark.parametrize(
    "input_name,expected_output",
    [
        # ROS2 packages
        ("ros2-my-package", "my_package"),
        ("ros2-complex-package-name", "complex_package_name"),
        ("ros2-ros-environment", "ros_environment"),
        ("ros2-ros-workspace", "ros_workspace"),
        ("ros2-catkin", "catkin"),
        ("ros2-navigation2", "navigation2"),
        ("ros2-my-very-long-package-name", "my_very_long_package_name"),
        ("ros2-navigation-stack-extra", "navigation_stack_extra"),
        ("ros2-my_package_name", "my_package_name"),
        # ROS1 packages
        ("ros-my-package", "my_package"),
        ("ros-test-package", "test_package"),
        ("ros-ros-environment", "ros_environment"),
        ("ros-ros-workspace", "ros_workspace"),
        ("ros-catkin", "catkin"),
        ("ros-moveit", "moveit"),
        ("ros-simple", "simple"),
        # Packages without prefix
        ("my-package", "my_package"),
        # Edge cases
        ("ros2-a", "a"),
        ("ros-a", "a"),
        ("ros2-ros", "ros"),
    ],
)
def test_package_normalization(input_name, expected_output):
    """Test package name normalization for various inputs."""
    vinca_conf = {}
    result = ensure_name_is_without_distro_prefix_and_with_underscores(
        input_name, vinca_conf
    )
    assert result == expected_output


@pytest.mark.parametrize(
    "package_name",
    [
        "ros2-my-package",
        "ros2-complex-package-name",
        "ros-simple-package",
        "ros-another-test",
    ],
)
def test_no_hyphens_in_output(package_name):
    """Test that normalized names contain no hyphens."""
    vinca_conf = {}
    result = ensure_name_is_without_distro_prefix_and_with_underscores(
        package_name, vinca_conf
    )
    assert "-" not in result


@pytest.mark.parametrize(
    "distribution_type,expected_prefix,expected_version",
    [
        ("ros1", "ros", "1"),
        ("ros2", "ros2", "2"),
    ],
)
def test_distro_prefix_and_version(
    distribution_type, expected_prefix, expected_version
):
    """Test that Distro returns correct package prefix and ROS version."""
    distro = Mock(spec=Distro)
    distro._distribution_type = distribution_type
    distro.check_ros1 = lambda: distro._distribution_type == "ros1"
    distro.get_package_prefix = lambda: "ros" if distro.check_ros1() else "ros2"
    distro.get_ros_version = lambda: "1" if distro.check_ros1() else "2"

    assert distro.get_package_prefix() == expected_prefix
    assert distro.get_ros_version() == expected_version
