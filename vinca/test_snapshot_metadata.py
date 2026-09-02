from unittest.mock import Mock, patch

import pytest

import vinca.main as main
from vinca.distro import Distro


def package_xml(name, version, build_dependency=None):
    dependency = (
        f"  <build_depend>{build_dependency}</build_depend>\n"
        if build_dependency
        else ""
    )
    return f"""\
<package format="3">
  <name>{name}</name>
  <version>{version}</version>
  <description>Test package</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
{dependency}  <export><build_type>ament_cmake</build_type></export>
</package>
"""


SNAPSHOT_PACKAGE_XML = package_xml("snapshot_package", "1.0.0", "snapshot_dependency")
LIVE_PACKAGE_XML = package_xml("snapshot_package", "2.0.0", "live_dependency")
SNAPSHOT_DEPENDENCY_XML = package_xml("snapshot_dependency", "1.0.0")


def make_snapshot_distro(monkeypatch):
    snapshot = {
        "snapshot_package": {
            "url": "https://github.com/example/snapshot-package-release.git",
            "version": "1.0.0",
            "tag": "release/rolling/snapshot_package/1.0.0-1",
            "dependencies": ["snapshot_dependency"],
        },
        "snapshot_dependency": {
            "url": "https://github.com/example/snapshot-dependency-release.git",
            "version": "1.0.0",
            "tag": "release/rolling/snapshot_dependency/1.0.0-1",
            "dependencies": [],
        },
    }
    distro = Distro.__new__(Distro)
    distro.distro_name = "rolling"
    distro.snapshot = snapshot
    distro.additional_packages_snapshot = None
    distro.build_packages = set()
    distro._distribution_type = "ros2"
    distro._additional_xml_cache = {}
    distro._depends_cache = {}
    distro._direct_depends_cache = {}
    distro._distro = Mock()
    snapshot_xml_by_url = {
        "https://raw.githubusercontent.com/example/snapshot-package-release/"
        "release/rolling/snapshot_package/1.0.0-1/package.xml": (SNAPSHOT_PACKAGE_XML),
        "https://raw.githubusercontent.com/example/snapshot-dependency-release/"
        "release/rolling/snapshot_dependency/1.0.0-1/package.xml": (
            SNAPSHOT_DEPENDENCY_XML
        ),
    }
    monkeypatch.setattr(
        distro,
        "_download_raw_pkg_xml_or_cached",
        lambda url: snapshot_xml_by_url[url],
    )
    distro._walker = Mock()
    return distro


def test_snapshot_package_xml_and_dependencies_do_not_follow_live_rosdistro(
    monkeypatch,
):
    distro = make_snapshot_distro(monkeypatch)

    package_xml_content = distro.get_release_package_xml("snapshot_package")

    assert distro.get_released_repo("snapshot_package") == (
        "https://github.com/example/snapshot-package-release.git",
        "release/rolling/snapshot_package/1.0.0-1",
        "tag",
    )
    assert distro.get_version("snapshot_package") == "1.0.0"
    assert "<version>1.0.0</version>" in package_xml_content
    assert "snapshot_dependency" in package_xml_content
    assert "live_dependency" not in package_xml_content
    assert distro.get_depends("snapshot_package") == {"snapshot_dependency"}
    distro._distro.get_release_package_xml.assert_not_called()
    distro._walker.get_depends.assert_not_called()


def test_snapshot_package_xml_uses_matching_live_distribution_cache(monkeypatch):
    distro = make_snapshot_distro(monkeypatch)
    release_repository = Mock(
        url="https://github.com/example/snapshot-package-release.git",
        version="1.0.0-1",
    )
    distro._distro.release_packages = {
        "snapshot_package": Mock(repository_name="snapshot-package")
    }
    distro._distro.repositories = {
        "snapshot-package": Mock(release_repository=release_repository)
    }
    distro._distro.get_release_package_xml.return_value = LIVE_PACKAGE_XML

    with patch(
        "vinca.distro.get_release_tag",
        return_value="release/rolling/snapshot_package/1.0.0-1",
    ):
        package_xml_content = distro.get_release_package_xml("snapshot_package")

    assert package_xml_content == LIVE_PACKAGE_XML
    distro._distro.get_release_package_xml.assert_called_once_with("snapshot_package")


def test_snapshot_package_xml_does_not_use_live_cache_after_snapshot_change(
    monkeypatch,
):
    distro = make_snapshot_distro(monkeypatch)
    release_repository = Mock(
        url="https://github.com/example/snapshot-package-release.git",
        version="2.0.0-1",
    )
    distro._distro.release_packages = {
        "snapshot_package": Mock(repository_name="snapshot-package")
    }
    distro._distro.repositories = {
        "snapshot-package": Mock(release_repository=release_repository)
    }

    with patch(
        "vinca.distro.get_release_tag",
        return_value="release/rolling/snapshot_package/2.0.0-1",
    ):
        package_xml_content = distro.get_release_package_xml("snapshot_package")

    assert package_xml_content == SNAPSHOT_PACKAGE_XML
    distro._distro.get_release_package_xml.assert_not_called()


def test_snapshot_without_dependencies_requires_regeneration(monkeypatch):
    distro = make_snapshot_distro(monkeypatch)
    del distro.snapshot["snapshot_package"]["dependencies"]

    with pytest.raises(RuntimeError, match="regenerate the rosdistro snapshot"):
        distro.get_depends("snapshot_package")


def test_snapshot_metadata_generates_dependency_required_by_pinned_source(
    monkeypatch,
):
    distro = make_snapshot_distro(monkeypatch)
    dependency_names = {
        "snapshot_package": "ros2-snapshot-package",
        "python": "python",
        "ament_cmake": "ros2-ament-cmake",
        "snapshot_dependency": "ros2-snapshot-dependency",
    }
    monkeypatch.setattr(
        main,
        "resolve_pkgname",
        lambda name, *_args, **_kwargs: [dependency_names[name]],
    )
    config = {
        "_selected_pkgs": {"snapshot_package"},
        "_pkg_additional_info": {},
        "depmods": {},
        "package_name_mode": "new",
    }

    output = main.generate_output(
        "snapshot_package",
        config,
        distro,
        distro.get_version("snapshot_package"),
    )

    assert output["package"] == {
        "name": "ros2-snapshot-package",
        "version": "1.0.0",
    }
    assert "ros2-snapshot-dependency" in output["requirements"]["host"]
    assert "ros2-live-dependency" not in output["requirements"]["host"]


def test_snapshot_is_authoritative_for_package_membership(monkeypatch):
    distro = make_snapshot_distro(monkeypatch)
    distro._distro.release_packages = {"live_only": Mock()}

    assert distro.check_package("snapshot_package")
    assert not distro.check_package("live_only")
    assert set(distro.get_package_names()) == {
        "snapshot_package",
        "snapshot_dependency",
    }


def test_empty_snapshot_keeps_live_rosdistro_behavior():
    distro = Distro.__new__(Distro)
    distro.snapshot = {}
    distro.additional_packages_snapshot = None
    distro.build_packages = set()
    distro._depends_cache = {}
    distro._direct_depends_cache = {}
    distro._distro = Mock()
    distro._distro.release_packages = {"live_package": Mock()}
    distro._distro.get_release_package_xml.return_value = LIVE_PACKAGE_XML
    distro._walker = Mock()
    distro._walker.get_depends.side_effect = (
        lambda package, dependency_type, ros_packages_only: (
            {"live_dependency"} if dependency_type == "run" else set()
        )
    )

    assert distro.check_package("live_package")
    assert distro.get_release_package_xml("live_package") == LIVE_PACKAGE_XML
    assert distro.get_depends("live_package") == {"live_dependency"}
    assert set(distro.get_package_names()) == {"live_package"}


def test_read_snapshot_merges_additional_packages(tmp_path, monkeypatch):
    snapshot_path = tmp_path / "rosdistro_snapshot.yaml"
    additional_path = tmp_path / "rosdistro_additional_recipes.yaml"
    snapshot_path.write_text(
        """\
snapshot_package:
  version: 1.0.0
"""
    )
    additional_path.write_text(
        """\
additional_package:
  version: 2.0.0
"""
    )
    monkeypatch.chdir(tmp_path)

    snapshot, additional = main.read_snapshot(
        {
            "rosdistro_snapshot": snapshot_path.name,
            "rosdistro_additional_recipes": additional_path.name,
        }
    )

    assert set(snapshot) == {"snapshot_package", "additional_package"}
    assert additional == {"additional_package": {"version": "2.0.0"}}
