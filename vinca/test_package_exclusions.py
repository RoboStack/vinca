"""Tests for packages_exclude / packages_skip and conda_index-shadowed packages."""

from typing import Any

import pytest

import vinca.main as m
from vinca.configuration import read_vinca_yaml
from vinca.resolve import should_skip_pkg


class FakeDistro:
    def __init__(self, depends, packages):
        self._depends = depends
        self._packages = set(packages)

    def check_ros1(self):
        return True  # keep the ROS 2 workspace packages out of the assertions

    def check_package(self, pkg):
        return pkg in self._packages

    def get_depends(self, pkg, ignore_pkgs=None):
        deps = set(self._depends.get(pkg, set()))
        if ignore_pkgs:
            deps -= set(ignore_pkgs)
        return deps


def _config(tmp_path, monkeypatch, body: str, platform: str = "linux-64"):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "patches").mkdir()
    (tmp_path / "vinca.yaml").write_text(
        "ros_distro: humble\nconda_index: []\npatch_dir: patches\n" + body
    )
    return read_vinca_yaml(tmp_path / "vinca.yaml", platform)


BODY = """\
packages_select_by_deps:
  - app
  - tool
  - viewer
  - helper
packages_exclude:
  - tool
  - if: win
    then:
      - viewer
packages_skip:
  - helper
"""


def test_exclude_and_skip_drop_selected_packages(tmp_path, monkeypatch):
    conf = _config(tmp_path, monkeypatch, BODY)

    assert conf["packages_select_by_deps"] == ["app", "viewer"]
    assert conf["packages_exclude"] == ["tool"]
    assert conf["packages_skip"] == ["helper"]


def test_exclude_under_selector_applies_on_that_platform_only(tmp_path, monkeypatch):
    conf = _config(tmp_path, monkeypatch, BODY, platform="win-64")

    assert conf["packages_select_by_deps"] == ["app"]
    assert conf["packages_exclude"] == ["tool", "viewer"]


def test_excluded_packages_are_dropped_from_dependencies(tmp_path, monkeypatch):
    conf = _config(tmp_path, monkeypatch, BODY)

    assert should_skip_pkg("tool", conf)
    assert not should_skip_pkg("helper", conf)  # skipped, but dependents keep it


def test_replaced_keys_still_work_with_a_warning(tmp_path, monkeypatch):
    body = """\
packages_select_by_deps:
  - app
  - tool
packages_skip_by_deps:
  - tool
  - helper
packages_remove_from_deps:
  - tool
  - removed
"""
    with pytest.warns(FutureWarning, match="packages_exclude"):
        conf = _config(tmp_path, monkeypatch, body)

    # as before: the lists don't change the selection, only traversal and dependencies
    assert conf["packages_select_by_deps"] == ["app", "tool"]
    assert conf["_skip_by_deps"] == ["tool", "helper"]
    assert should_skip_pkg("tool", conf)
    assert should_skip_pkg("removed", conf)
    assert not should_skip_pkg("helper", conf)
    assert "packages_skip_by_deps" not in conf


def test_conda_index_shadowed_ros_packages_are_not_built():
    distro = FakeDistro(
        {"app": {"tl_expected", "libA"}},
        packages={"app", "libA", "tl_expected"},
    )
    conf: dict[str, Any] = {
        "packages_select_by_deps": ["app", "tl_expected"],
        "packages_skip": None,
        "_conda_indexes": [
            {"tl_expected": {"robostack": ["cpp-expected"]}, "eigen": {}}
        ],
    }

    selected = m.get_selected_packages(distro, conf)

    assert set(selected) == {"app", "libA"}
    assert m.conda_index_shadowed_packages(distro, conf) == {"tl_expected"}
