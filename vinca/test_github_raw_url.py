from typing import Any

from vinca.distro import Distro


def _distro() -> Any:
    return Distro.__new__(Distro)


def test_tag_ref_uses_explicit_refs_tags_prefix():
    # ros2-gbp release tags look like "release/jazzy/foo_pkg/1.2.3-1" -- the
    # short <owner>/<repo>/<ref>/<path> raw.githubusercontent.com form has to
    # guess where a slash-containing ref ends and the path begins, and that
    # guess is inconsistently cached across CDN edges (the same URL 404s from
    # some vantage points, including GitHub Actions runners, while resolving
    # fine from others). The explicit refs/tags/<name> form is unambiguous.
    pkg_info = {
        "url": "https://github.com/ros2-gbp/ros2_control-release.git",
        "tag": "release/jazzy/controller_interface/4.47.0-1",
    }

    url = _distro()._construct_raw_url_github(pkg_info)

    assert url == (
        "https://raw.githubusercontent.com/ros2-gbp/ros2_control-release/"
        "refs/tags/release/jazzy/controller_interface/4.47.0-1/package.xml"
    )


def test_rev_ref_is_used_as_is():
    # A commit hash is already unambiguous -- it must not get the refs/tags/
    # prefix, since it isn't a tag name.
    pkg_info = {
        "url": "https://github.com/ros2-gbp/ros2_control-release.git",
        "rev": "abc123def456",
    }

    url = _distro()._construct_raw_url_github(pkg_info)

    assert url == (
        "https://raw.githubusercontent.com/ros2-gbp/ros2_control-release/"
        "abc123def456/package.xml"
    )


def test_tag_ref_with_additional_folder_and_custom_xml_name():
    pkg_info = {
        "url": "https://github.com/example/some-release.git",
        "tag": "release/rolling/some_pkg/1.0.0-1",
        "additional_folder": "some_pkg",
        "package_xml_name": "package.xml",
    }

    url = _distro()._construct_raw_url_github(pkg_info)

    assert url == (
        "https://raw.githubusercontent.com/example/some-release/"
        "refs/tags/release/rolling/some_pkg/1.0.0-1/some_pkg/package.xml"
    )
