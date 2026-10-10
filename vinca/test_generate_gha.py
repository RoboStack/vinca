"""Tests for the GitHub Actions pipeline generation."""

import pytest

from vinca import config
from vinca.generate_gha import (
    build_unix_pipeline,
    build_win_pipeline,
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
    steps = workflow["jobs"]["stage_0_job_0"]["steps"]
    assert steps[1]["name"] == "Disable git auto-maintenance"
    assert "maintenance.auto false" in steps[1]["run"]
    setup_step = steps[2]
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


def test_win_pipeline_inlines_the_repository_build_script(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / ".scripts").mkdir()
    (tmp_path / ".scripts" / "build_win.bat").write_text("echo repository script\n")
    outfile = tmp_path / "win.yml"

    build_win_pipeline([[["ros2-rclcpp"]]], "buildbranch_win", outfile=outfile)

    workflow = pytest.importorskip("yaml").safe_load(outfile.read_text())
    steps = workflow["jobs"]["stage_0_job_0"]["steps"]
    assert "echo repository script" in steps[-1]["run"]


def test_win_pipeline_requires_the_repository_build_script(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)

    with pytest.raises(FileNotFoundError, match="build_win.bat"):
        build_win_pipeline(
            [[["ros2-rclcpp"]]], "buildbranch_win", outfile=tmp_path / "win.yml"
        )


def test_only_later_stages_declare_needs(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / ".scripts").mkdir()
    (tmp_path / ".scripts" / "build_win.bat").write_text("echo build\n")
    stages = [[["ros2-a"], ["ros2-b"]], [["ros2-c"]]]
    yaml = pytest.importorskip("yaml")

    build_unix_pipeline(stages, "branch", outfile=tmp_path / "linux.yml")
    build_win_pipeline(stages, "branch", outfile=tmp_path / "win.yml")

    for outfile in ("linux.yml", "win.yml"):
        jobs = yaml.safe_load((tmp_path / outfile).read_text())["jobs"]
        # actionlint rejects `needs: []`
        assert "needs" not in jobs["stage_0_job_0"]
        assert "needs" not in jobs["stage_0_job_1"]
        assert jobs["stage_1_job_2"]["needs"] == ["stage_0_job_0", "stage_0_job_1"]
