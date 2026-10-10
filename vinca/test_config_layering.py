"""Tests for ``extends:``: a configuration on top of a shared base configuration."""

import pytest

from vinca.configuration import read_vinca_yaml


def _write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


@pytest.fixture
def layered(tmp_path, monkeypatch):
    shared, distro = tmp_path / "shared", tmp_path / "distros" / "humble"
    _write(
        shared / "vinca.yaml",
        """\
package_name_mode: new
conda_index: [robostack.yaml]
patch_dir: patch
skip_testing: true
mutex_package:
  name: ros2-distro-mutex
  upper_bound: x.x
  run_constraints:
    - libboost 1.90.*
    - pcl 1.15.*
packages_select_by_deps:
  - desktop
  - if: win
    then: [win_only]
packages_exclude:
  - broken
""",
    )
    _write(
        shared / "robostack.yaml",
        "eigen:\n  robostack: [eigen]\nfoo:\n  robostack: [foo]\n",
    )
    _write(
        shared / "patch" / "dependencies.yaml",
        "demo:\n  add_host: [shared-dep]\n  add_run: [x]\n",
    )
    _write(shared / "patch" / "ros2-demo.patch", "shared patch")
    _write(shared / "patch" / "ros2-other.patch", "shared other patch")
    _write(
        shared / "pkg_additional_info.yaml",
        "demo:\n  additional_cmake_args: -DFOO=ON\n",
    )
    _write(shared / "tests" / "ros2-demo.yaml", "tests: [shared]\n")
    _write(shared / "tests" / "ros2-other.yaml", "tests: [shared]\n")

    _write(
        distro / "vinca.yaml",
        """\
extends: ../../shared/vinca.yaml
ros_distro: humble
conda_index: [robostack.yaml]
patch_dir: patch
mutex_package:
  version: 0.10.0
  run_constraints:
    - pcl 1.16.*
    - krb5 1.22.*
packages_select_by_deps:
  - extra
packages_skip:
  - desktop
""",
    )
    _write(distro / "robostack.yaml", "foo:\n  robostack: [foo-own]\n")
    _write(distro / "patch" / "dependencies.yaml", "demo:\n  add_run: [own-dep]\n")
    _write(distro / "patch" / "ros2-demo.patch", "own patch")
    _write(distro / "pkg_additional_info.yaml", "demo:\n  build_number: 3\n")
    _write(distro / "tests" / "ros2-demo.yaml", "tests: [own]\n")

    monkeypatch.chdir(distro)
    return shared, distro


def test_lists_and_scalars_are_layered(layered):
    conf = read_vinca_yaml(layered[1] / "vinca.yaml", "linux-64")

    assert conf["packages_select_by_deps"] == ["extra"]  # desktop is skipped
    assert conf["packages_skip"] == ["desktop"]
    assert conf["packages_exclude"] == ["broken"]
    assert conf["package_name_mode"] == "new"
    assert conf["skip_testing"] is True
    assert "extends" not in conf


def test_selectors_of_the_base_are_resolved(layered):
    conf = read_vinca_yaml(layered[1] / "vinca.yaml", "win-64")

    assert conf["packages_select_by_deps"] == ["win_only", "extra"]


def test_mutex_constraints_are_overridden_by_package(layered):
    mutex = read_vinca_yaml(layered[1] / "vinca.yaml", "linux-64")["mutex_package"]

    assert mutex["name"] == "ros2-distro-mutex"
    assert mutex["version"] == "0.10.0"
    assert mutex["upper_bound"] == "x.x"
    assert mutex["run_constraints"] == ["libboost 1.90.*", "pcl 1.16.*", "krb5 1.22.*"]


def test_files_of_each_layer(layered):
    shared, distro = layered
    conf = read_vinca_yaml(distro / "vinca.yaml", "linux-64")

    # per-package settings: the distribution's keys win
    assert conf["depmods"]["demo"] == {
        "add_host": ["shared-dep"],
        "add_run": ["own-dep"],
    }
    assert conf["_pkg_additional_info"]["demo"] == {
        "additional_cmake_args": "-DFOO=ON",
        "build_number": 3,
    }
    # patches and tests: the distribution's file replaces the shared one
    assert conf["_patches"]["ros2-demo"]["any"] == [
        str(distro / "patch" / "ros2-demo.patch")
    ]
    assert conf["_patches"]["ros2-other"]["any"] == [
        str(shared / "patch" / "ros2-other.patch")
    ]
    assert conf["_tests"]["ros2-demo"] == distro / "tests" / "ros2-demo.yaml"
    assert conf["_tests"]["ros2-other"] == shared / "tests" / "ros2-other.yaml"
    # conda_index: the distribution's mappings are looked up first
    assert [list(index) for index in conf["_conda_indexes"]] == [
        ["foo"],
        ["eigen", "foo"],
    ]


def test_extends_cycle_is_rejected(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    _write(tmp_path / "a.yaml", "extends: b.yaml\n")
    _write(tmp_path / "b.yaml", "extends: a.yaml\n")

    with pytest.raises(ValueError, match="extends itself"):
        read_vinca_yaml(tmp_path / "a.yaml", "linux-64")


def test_paths_are_relative_to_the_file_that_sets_them(layered, tmp_path, monkeypatch):
    shared, distro = layered
    _write(distro / "rosdistro_snapshot.yaml", "demo:\n  version: 1.0.0\n")
    text = (distro / "vinca.yaml").read_text()
    (distro / "vinca.yaml").write_text(
        text + "rosdistro_snapshot: rosdistro_snapshot.yaml\n"
    )
    monkeypatch.chdir(tmp_path)  # not the configuration's directory

    conf = read_vinca_yaml(distro / "vinca.yaml", "linux-64")

    assert conf["_snapshot"]["demo"]["version"] == "1.0.0"
    assert conf["_patch_dir"] == distro / "patch"
    assert conf["_patches"]["ros2-demo"]["any"] == [
        str(distro / "patch" / "ros2-demo.patch")
    ]
