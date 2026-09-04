from typing import Any
from unittest.mock import Mock, patch

import vinca.main as main
import vinca.recipes as recipes
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
            "repository": "https://github.com/example/snapshot-package.git",
            "version": "1.0.0",
            "tag": "release/rolling/snapshot_package/1.0.0-1",
        },
        "snapshot_dependency": {
            "url": "https://github.com/example/snapshot-dependency-release.git",
            "repository": "https://github.com/example/snapshot-dependency.git",
            "version": "1.0.0",
            "tag": "release/rolling/snapshot_dependency/1.0.0-1",
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
    distro._distro.get_release_package_xml.return_value = LIVE_PACKAGE_XML
    distro._walker = Mock()

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
    return distro


def test_snapshot_package_xml_and_dependencies_do_not_follow_live_rosdistro(
    monkeypatch,
):
    distro: Any = make_snapshot_distro(monkeypatch)

    package_xml_content = distro.get_release_package_xml("snapshot_package")

    assert distro.get_released_repo("snapshot_package") == (
        "https://github.com/example/snapshot-package-release.git",
        "release/rolling/snapshot_package/1.0.0-1",
        "tag",
    )
    assert (
        distro.get_repository_url("snapshot_package")
        == "https://github.com/example/snapshot-package"
    )
    assert distro.get_version("snapshot_package") == "1.0.0"
    assert package_xml_content is not None

    assert "<version>1.0.0</version>" in package_xml_content
    assert "snapshot_dependency" in package_xml_content
    assert "live_dependency" not in package_xml_content
    assert distro.get_depends("snapshot_package") == {"snapshot_dependency"}
    distro._distro.get_release_package_xml.assert_not_called()
    distro._walker.get_recursive_depends.assert_not_called()


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


def test_snapshot_metadata_generates_dependency_required_by_pinned_source(
    monkeypatch,
):
    distro: Any = make_snapshot_distro(monkeypatch)
    dependency_names = {
        "snapshot_package": "ros2-snapshot-package",
        "python": "python",
        "ament_cmake": "ros2-ament-cmake",
        "snapshot_dependency": "ros2-snapshot-dependency",
    }
    monkeypatch.setattr(
        recipes,
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
    assert output is not None

    assert output["package"] == {
        "name": "ros2-snapshot-package",
        "version": "1.0.0",
    }
    assert output["about"]["repository"] == (
        "https://github.com/example/snapshot-package"
    )
    assert "ros2-snapshot-dependency" in output["requirements"]["host"]
    assert "ros2-live-dependency" not in output["requirements"]["host"]


def test_snapshot_is_authoritative_for_package_membership(monkeypatch):
    distro: Any = make_snapshot_distro(monkeypatch)
    distro._distro.release_packages = {"live_only": Mock()}

    assert distro.check_package("snapshot_package")
    assert not distro.check_package("live_only")
    assert set(distro.get_package_names()) == {
        "snapshot_package",
        "snapshot_dependency",
    }


def test_live_repository_url_requires_upstream_source_metadata():
    distro = Distro.__new__(Distro)
    distro.snapshot = None
    distro.additional_packages_snapshot = None
    distro._distro = Mock()
    distro._distro.release_packages = {
        "source_package": Mock(repository_name="source-package"),
        "release_only_package": Mock(repository_name="release-only-package"),
        "bloom_source_package": Mock(repository_name="bloom-source-package"),
    }
    distro._distro.repositories = {
        "source-package": Mock(
            source_repository=Mock(url="https://github.com/example/source.git"),
            release_repository=Mock(
                url="https://github.com/example/source-release.git"
            ),
        ),
        "release-only-package": Mock(
            source_repository=None,
            release_repository=Mock(
                url="https://github.com/example/release-only-release.git"
            ),
        ),
        "bloom-source-package": Mock(
            source_repository=Mock(
                url="https://github.com/example/bloom-source-release.git"
            ),
            release_repository=Mock(
                url="https://github.com/ros2-gbp/bloom-source-release.git"
            ),
        ),
    }

    assert (
        distro.get_repository_url("source_package")
        == "https://github.com/example/source"
    )
    assert distro.get_repository_url("release_only_package") is None
    assert (
        distro.get_repository_url(
            "release_only_package",
            [Mock(type="repository", url="https://github.com/example/upstream.git")],
        )
        == "https://github.com/example/upstream"
    )
    assert distro.get_repository_url("bloom_source_package") is None


def test_additional_package_repository_must_be_explicit():
    distro = Distro.__new__(Distro)
    distro.snapshot = None
    distro.additional_packages_snapshot = {
        "explicit": {
            "url": "https://github.com/example/explicit-release.git",
            "repository": "https://github.com/example/explicit.git",
        },
        "source_only": {"url": "https://github.com/example/source-only.git"},
    }

    assert (
        distro.get_repository_url("explicit") == "https://github.com/example/explicit"
    )
    assert distro.get_repository_url("source_only") is None
    assert (
        distro.get_repository_url(
            "source_only",
            [Mock(type="repository", url="https://github.com/example/source-only.git")],
        )
        == "https://github.com/example/source-only"
    )
    assert (
        distro.get_repository_url(
            "source_only",
            [
                Mock(
                    type="repository",
                    url="https://github.com/example/source-only-release.git",
                )
            ],
        )
        is None
    )


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
