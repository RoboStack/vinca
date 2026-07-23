"""Tests for the ROS package naming scheme."""

import pytest

from vinca.distro import Distro
from vinca.main import append_output_with_compatibility, get_depmods
from vinca.naming import (
    PackageNameMode,
    generate_legacy_compatibility_output,
    get_package_name,
    get_package_name_mode,
)
from vinca.resolve import resolve_pkgname
from vinca.utils import (
    add_package_name_variants,
    ensure_name_is_without_distro_prefix_and_with_underscores,
)


def make_distro(distribution_type="ros2", name="humble"):
    """Create a Distro without loading the remote rosdistro index."""
    distro = Distro.__new__(Distro)
    distro._distribution_type = distribution_type
    distro.distro_name = name
    return distro


@pytest.mark.parametrize(
    "input_name,expected_output",
    [
        # New ROS 2 package names
        ("ros2-my-package", "my_package"),
        ("ros2-complex-package-name", "complex_package_name"),
        ("ros2-ros-environment", "ros_environment"),
        ("ros2-ros-workspace", "ros_workspace"),
        ("ros2-catkin", "catkin"),
        ("ros2-navigation2", "navigation2"),
        ("ros2-my-very-long-package-name", "my_very_long_package_name"),
        ("ros2-navigation-stack-extra", "navigation_stack_extra"),
        ("ros2-my_package_name", "my_package_name"),
        # New ROS 1 package names
        ("ros-my-package", "my_package"),
        ("ros-test-package", "test_package"),
        ("ros-ros-environment", "ros_environment"),
        ("ros-ros-workspace", "ros_workspace"),
        ("ros-catkin", "catkin"),
        ("ros-moveit", "moveit"),
        ("ros-simple", "simple"),
        # Legacy distro-qualified package names
        ("ros-humble-my-package", "my_package"),
        ("ros-humble-ros-workspace", "ros_workspace"),
        # Packages without a prefix
        ("my-package", "my_package"),
        # Edge cases
        ("ros2-a", "a"),
        ("ros-a", "a"),
        ("ros2-ros", "ros"),
    ],
)
def test_package_normalization(input_name, expected_output):
    result = ensure_name_is_without_distro_prefix_and_with_underscores(
        input_name, {"ros_distro": "humble"}
    )
    assert result == expected_output


@pytest.mark.parametrize(
    "package_name",
    [
        "ros2-my-package",
        "ros2-complex-package-name",
        "ros-simple-package",
        "ros-another-test",
        "ros-humble-legacy-package",
    ],
)
def test_no_hyphens_in_normalized_output(package_name):
    result = ensure_name_is_without_distro_prefix_and_with_underscores(
        package_name, {"ros_distro": "humble"}
    )
    assert "-" not in result


@pytest.mark.parametrize(
    "distribution_type,distro_name,expected_prefix,expected_legacy_prefix,expected_version",
    [
        ("ros1", "noetic", "ros", "ros-noetic", "1"),
        ("ros2", "humble", "ros2", "ros-humble", "2"),
    ],
)
def test_distro_prefixes_and_version(
    distribution_type,
    distro_name,
    expected_prefix,
    expected_legacy_prefix,
    expected_version,
):
    distro = make_distro(distribution_type, distro_name)

    assert distro.get_package_prefix() == expected_prefix
    assert distro.get_legacy_package_prefix() == expected_legacy_prefix
    assert distro.get_ros_version() == expected_version


@pytest.mark.parametrize(
    "mode,expected_name",
    [
        ("legacy", "ros-humble-my-package"),
        ("both", "ros2-my-package"),
        ("new", "ros2-my-package"),
    ],
)
def test_package_name_mode_selects_primary_name(mode, expected_name):
    distro = make_distro()
    assert (
        get_package_name("my_package", distro, {"package_name_mode": mode})
        == expected_name
    )


def test_package_name_mode_defaults_to_legacy():
    distro = make_distro()

    assert get_package_name_mode({}) is PackageNameMode.LEGACY
    assert get_package_name("my_package", distro, {}) == "ros-humble-my-package"


@pytest.mark.parametrize(
    "mode,expected_name",
    [
        ("legacy", "ros-humble-my-package"),
        ("both", "ros2-my-package"),
        ("new", "ros2-my-package"),
    ],
)
def test_dependency_resolution_uses_selected_name_mode(mode, expected_name):
    distro = make_distro()
    distro.check_package = lambda _name: True
    vinca_conf = {"_conda_indexes": [], "package_name_mode": mode}

    assert resolve_pkgname("my_package", vinca_conf, distro) == [expected_name]


@pytest.mark.parametrize(
    "skip_built_packages,expected_names",
    [
        (set(), ["ros2-my-package", "ros-humble-my-package"]),
        ({"ros2-my-package"}, ["ros-humble-my-package"]),
        ({"ros-humble-my-package"}, ["ros2-my-package"]),
        ({"ros2-my-package", "ros-humble-my-package"}, []),
    ],
)
def test_both_mode_filters_new_and_legacy_outputs_independently(
    skip_built_packages, expected_names
):
    distro = make_distro()
    output = {"package": {"name": "ros2-my-package", "version": "1.2.3"}}
    outputs = []
    vinca_conf = {
        "package_name_mode": "both",
        "skip_built_packages": skip_built_packages,
    }

    append_output_with_compatibility(outputs, output, "my_package", distro, vinca_conf)

    assert [candidate["package"]["name"] for candidate in outputs] == expected_names


def test_version_aware_skip_filters_compatibility_output_independently():
    distro = make_distro()
    output = {"package": {"name": "ros2-my-package", "version": "1.2.3"}}
    outputs = []
    vinca_conf = {
        "package_name_mode": "both",
        "trigger_new_versions": True,
        "skip_built_packages": {("ros-humble-my-package", "1.2.3")},
    }

    append_output_with_compatibility(outputs, output, "my_package", distro, vinca_conf)

    assert [candidate["package"]["name"] for candidate in outputs] == [
        "ros2-my-package"
    ]


def test_depmods_use_selected_name_mode_recursively():
    distro = make_distro()
    vinca_conf = {
        "package_name_mode": "new",
        "depmods": {
            "my_package": {
                "add_host": [
                    "ros-humble-rclcpp >=1",
                    "ros-noetic-roscpp",
                    {
                        "if": "target_platform == 'emscripten-wasm32'",
                        "then": ["ros-humble-rmw-wasm-cpp"],
                    },
                ],
                "remove_run": ["ros-humble-tl-expected"],
            }
        },
    }

    removed, added = get_depmods(vinca_conf, "my_package", distro)

    assert added["host"] == [
        "ros2-rclcpp >=1",
        "ros-noetic-roscpp",
        {
            "if": "target_platform == 'emscripten-wasm32'",
            "then": ["ros2-rmw-wasm-cpp"],
        },
    ]
    assert removed["run"] == ["ros2-tl-expected"]


def test_package_config_entries_are_available_under_old_and_new_names():
    value = object()
    entries = {"ros-humble-my-package": value}

    add_package_name_variants(entries, "humble")

    assert entries["ros-humble-my-package"] is value
    assert entries["ros-my-package"] is value
    assert entries["ros2-my-package"] is value


def test_invalid_package_name_mode_is_rejected():
    with pytest.raises(ValueError, match="Invalid package_name_mode 'invalid'"):
        get_package_name_mode({"package_name_mode": "invalid"})


def test_both_mode_generates_legacy_compatibility_output():
    distro = make_distro()
    output = {"package": {"name": "ros2-my-package", "version": "1.2.3"}}

    compatibility_output = generate_legacy_compatibility_output(
        output, "my_package", distro, {"package_name_mode": "both"}
    )

    assert compatibility_output == {
        "package": {"name": "ros-humble-my-package", "version": "1.2.3"},
        "build": {"script": ""},
        "requirements": {"run": ["ros2-my-package ==1.2.3"]},
        "about": {"summary": "Compatibility package for ros2-my-package"},
    }


@pytest.mark.parametrize("mode", ["legacy", "new"])
def test_single_name_modes_do_not_generate_compatibility_output(mode):
    distro = make_distro()
    output_name = get_package_name("my_package", distro, {"package_name_mode": mode})
    output = {"package": {"name": output_name, "version": "1.2.3"}}

    assert (
        generate_legacy_compatibility_output(
            output, "my_package", distro, {"package_name_mode": mode}
        )
        is None
    )
