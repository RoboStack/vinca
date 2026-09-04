"""Tests for the GitHub Actions pipeline generation."""

import pytest

from vinca import config
from vinca.generate_gha import (
    build_unix_pipeline,
    get_setup_pixi_step,
    get_stage_name,
)


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


def test_unix_pipeline_sets_up_pixi(tmp_path):
    outfile = tmp_path / "linux.yml"

    build_unix_pipeline(
        [[["ros-rolling-rclcpp"]]],
        "buildbranch_linux",
        script="build command",
        outfile=outfile,
        target="linux-64",
    )

    workflow = pytest.importorskip("yaml").safe_load(outfile.read_text())
    setup_step = workflow["jobs"]["stage_0_job_0"]["steps"][1]
    assert setup_step == {
        "name": "Setup pixi",
        "uses": "prefix-dev/setup-pixi@v0",
        "with": {
            "pixi-version": "latest",
            "cache": "true",
            "log-level": "v",
            "frozen": "true",
        },
    }


@pytest.mark.parametrize(
    "configured_version,expected_ref",
    [
        ("latest", "latest"),
        ("0.10.2", "v0.10.2"),
        ("v0.10.2", "v0.10.2"),
        ("  v0.10.2  ", "v0.10.2"),
        ("0.10.2-rc.1", "v0.10.2-rc.1"),
        (
            "0123456789abcdef0123456789abcdef01234567",
            "0123456789abcdef0123456789abcdef01234567",
        ),
    ],
)
def test_setup_pixi_version_can_be_configured(configured_version, expected_ref):
    step = get_setup_pixi_step(setup_pixi_version=configured_version)
    assert step["uses"] == f"prefix-dev/setup-pixi@{expected_ref}"


@pytest.mark.parametrize(
    "configured_version,expected_version",
    [("latest", "latest"), ("0.78.0", "v0.78.0"), ("v0.78.0", "v0.78.0")],
)
def test_pixi_version_can_be_configured(configured_version, expected_version):
    step = get_setup_pixi_step(pixi_version=configured_version)
    assert step["with"]["pixi-version"] == expected_version


def test_setup_pixi_versions_must_not_be_empty():
    with pytest.raises(ValueError, match="must not be empty"):
        get_setup_pixi_step(setup_pixi_version="  ")
