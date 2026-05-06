"""Tests for parsing pixi.toml manifests as catkin_pkg.Package objects."""

from collections.abc import Iterable
from pathlib import Path

import pytest
from catkin_pkg.package import Dependency, Package

from vinca.pixi_manifest import (
    parse_additional_manifest,
    parse_pixi_package,
    parse_pixi_package_string,
)

FIXTURES = Path(__file__).parent / "test_fixtures" / "pixi_manifests"


def _dep_names(deps: Iterable[Dependency]) -> list[str]:
    return sorted(d.name for d in deps)


def test_parses_name_and_version() -> None:
    pkg = parse_pixi_package(FIXTURES / "minimal" / "pixi.toml", ros_distro="kilted")

    assert pkg.name == "minimal_pkg"
    assert pkg.version == "1.2.3"


def test_parses_description_and_license() -> None:
    pkg = parse_pixi_package(FIXTURES / "minimal" / "pixi.toml", ros_distro="kilted")

    assert pkg.description == "A minimal package"
    assert [str(lic) for lic in pkg.licenses] == ["MIT"]


def test_parses_authors_and_maintainers_from_authors_field() -> None:
    pkg = parse_pixi_package(FIXTURES / "minimal" / "pixi.toml", ros_distro="kilted")

    # The pixi.toml has a single "Alice <alice@example.com>" author.
    # We mirror it into both maintainers and authors so package.xml-shaped
    # downstream code (which usually wants both) sees the same person.
    assert len(pkg.authors) == 1
    assert pkg.authors[0].name == "Alice"
    assert pkg.authors[0].email == "alice@example.com"

    assert len(pkg.maintainers) == 1
    assert pkg.maintainers[0].name == "Alice"
    assert pkg.maintainers[0].email == "alice@example.com"


def test_build_type_becomes_export() -> None:
    pkg = parse_pixi_package(FIXTURES / "minimal" / "pixi.toml", ros_distro="kilted")

    build_type_exports = [e for e in pkg.exports if e.tagname == "build_type"]
    assert len(build_type_exports) == 1
    assert build_type_exports[0].content == "ament_python"


def test_host_only_deps_become_build_depends() -> None:
    pkg = parse_pixi_package(FIXTURES / "with_deps" / "pixi.toml", ros_distro="kilted")

    # `cmake` is host-only → build_depends. ros prefix is stripped.
    assert _dep_names(pkg.build_depends) == ["ament_cmake", "cmake"]


def test_run_only_deps_become_exec_depends() -> None:
    pkg = parse_pixi_package(FIXTURES / "with_deps" / "pixi.toml", ros_distro="kilted")

    assert _dep_names(pkg.exec_depends) == ["psutil", "rclcpp"]


def test_test_dependencies_become_test_depends() -> None:
    pkg = parse_pixi_package(FIXTURES / "with_deps" / "pixi.toml", ros_distro="kilted")

    assert _dep_names(pkg.test_depends) == ["ament_pytest", "pytest"]


def test_test_dependencies_carry_version_specs() -> None:
    pkg = parse_pixi_package(FIXTURES / "with_deps" / "pixi.toml", ros_distro="kilted")
    by_name = {d.name: d for d in pkg.test_depends}

    assert by_name["pytest"].version_gte is None
    assert by_name["ament_pytest"].version_gte == "0.14"


def test_dep_evaluated_condition_set_so_vinca_keeps_them() -> None:
    pkg = parse_pixi_package(FIXTURES / "with_deps" / "pixi.toml", ros_distro="kilted")

    # vinca filters deps where `evaluated_condition` is falsy; the shim
    # must mark every dep as kept.
    for dep in (*pkg.build_depends, *pkg.exec_depends, *pkg.test_depends):
        assert dep.evaluated_condition is True, dep.name


def test_dep_in_both_host_and_run_appears_in_both_slots() -> None:
    fixture = FIXTURES / "shared_dep" / "pixi.toml"
    fixture.parent.mkdir(parents=True, exist_ok=True)
    fixture.write_text(
        '[package]\n'
        'name = "shared_dep"\n'
        'version = "0.1.0"\n'
        '[package.host-dependencies]\n'
        'ros-kilted-rclcpp = "*"\n'
        '[package.run-dependencies]\n'
        'ros-kilted-rclcpp = "*"\n'
        '[package.metadata.ros]\n'
        'build_type = "ament_cmake"\n'
    )
    pkg: Package = parse_pixi_package(fixture, ros_distro="kilted")

    assert _dep_names(pkg.build_depends) == ["rclcpp"]
    assert _dep_names(pkg.exec_depends) == ["rclcpp"]


def test_is_message_package_adds_buildtool_and_group_membership() -> None:
    pkg = parse_pixi_package(FIXTURES / "message_pkg" / "pixi.toml", ros_distro="kilted")

    # Message packages need rosidl_default_generators as a buildtool depend so
    # the rosidl generators participate in the build.
    assert "rosidl_default_generators" in _dep_names(pkg.buildtool_depends)
    for dep in pkg.buildtool_depends:
        assert dep.evaluated_condition is True

    # And membership in the rosidl_interface_packages group so generators
    # (which group_depend on it) discover this package.
    assert [g.name for g in pkg.member_of_groups] == ["rosidl_interface_packages"]
    for g in pkg.member_of_groups:
        assert g.evaluated_condition is True


def test_unconstrained_dep_has_no_version_fields() -> None:
    fixture = FIXTURES / "version_specs" / "pixi.toml"
    fixture.parent.mkdir(parents=True, exist_ok=True)
    fixture.write_text(
        '[package]\n'
        'name = "vs"\n'
        'version = "0.1.0"\n'
        '[package.run-dependencies]\n'
        'unbounded = "*"\n'
        'pinned = "==1.2.3"\n'
        'ranged = ">=2.5,<3"\n'
        'just_lower = ">1.0"\n'
        'just_upper = "<=4"\n'
        '[package.metadata.ros]\n'
        'build_type = "ament_python"\n'
    )
    pkg = parse_pixi_package(fixture, ros_distro="kilted")
    by_name = {d.name: d for d in pkg.exec_depends}

    # unbounded → no version constraints set
    u = by_name["unbounded"]
    assert u.version_eq is None
    assert u.version_gte is None and u.version_gt is None
    assert u.version_lte is None and u.version_lt is None


def test_exact_pin_sets_version_eq() -> None:
    pkg = parse_pixi_package(FIXTURES / "version_specs" / "pixi.toml", ros_distro="kilted")
    pinned = next(d for d in pkg.exec_depends if d.name == "pinned")
    assert pinned.version_eq == "1.2.3"


def test_compound_range_sets_both_bounds() -> None:
    pkg = parse_pixi_package(FIXTURES / "version_specs" / "pixi.toml", ros_distro="kilted")
    ranged = next(d for d in pkg.exec_depends if d.name == "ranged")
    assert ranged.version_gte == "2.5"
    assert ranged.version_lt == "3"


def test_strict_inequalities_use_gt_lt() -> None:
    pkg = parse_pixi_package(FIXTURES / "version_specs" / "pixi.toml", ros_distro="kilted")
    just_lower = next(d for d in pkg.exec_depends if d.name == "just_lower")
    just_upper = next(d for d in pkg.exec_depends if d.name == "just_upper")
    assert just_lower.version_gt == "1.0"
    assert just_lower.version_gte is None
    assert just_upper.version_lte == "4"
    assert just_upper.version_lt is None


def test_unsupported_version_spec_raises() -> None:
    fixture = FIXTURES / "bad_version" / "pixi.toml"
    fixture.parent.mkdir(parents=True, exist_ok=True)
    fixture.write_text(
        '[package]\n'
        'name = "bad"\n'
        'version = "0.1.0"\n'
        '[package.run-dependencies]\n'
        'wildcarded = "1.2.*"\n'
        '[package.metadata.ros]\n'
        'build_type = "ament_python"\n'
    )
    with pytest.raises(ValueError, match="wildcarded"):
        parse_pixi_package(fixture, ros_distro="kilted")


def test_parse_string_round_trips_to_same_package() -> None:
    """Parsing from a string should yield the same Package as parsing from the file."""
    path = FIXTURES / "with_deps" / "pixi.toml"
    from_path = parse_pixi_package(path, ros_distro="kilted")
    from_string = parse_pixi_package_string(
        path.read_text(), ros_distro="kilted", source="from_test"
    )

    assert from_string.name == from_path.name
    assert from_string.version == from_path.version
    assert _dep_names(from_string.exec_depends) == _dep_names(from_path.exec_depends)
    assert _dep_names(from_string.build_depends) == _dep_names(from_path.build_depends)


def test_parse_string_uses_source_in_errors() -> None:
    """When parsing fails, error messages should reference the supplied source label."""
    bad = '[package]\nname = "x"\nversion = "0.0.1"\n[package.metadata.ros]\n'
    with pytest.raises(KeyError, match="github.com/foo/bar"):
        parse_pixi_package_string(bad, ros_distro="kilted", source="github.com/foo/bar")


def test_dispatch_pixi_toml_uses_shim() -> None:
    content = (FIXTURES / "with_deps" / "pixi.toml").read_text()
    pkg = parse_additional_manifest(
        "pixi.toml", content, ros_distro="kilted", source="github.com/foo/bar"
    )
    assert pkg.name == "with_deps"
    assert "rclcpp" in [d.name for d in pkg.exec_depends]


def test_dispatch_package_xml_uses_catkin_pkg() -> None:
    xml = (
        '<?xml version="1.0"?>\n'
        '<package format="3">\n'
        '  <name>some_pkg</name>\n'
        '  <version>0.1.0</version>\n'
        '  <description>x</description>\n'
        '  <maintainer email="alice@example.com">A</maintainer>\n'
        '  <license>MIT</license>\n'
        '  <exec_depend>rclcpp</exec_depend>\n'
        '  <export><build_type>ament_cmake</build_type></export>\n'
        '</package>\n'
    )
    pkg = parse_additional_manifest(
        "package.xml", xml, ros_distro="kilted", source="github.com/foo/bar"
    )
    assert pkg.name == "some_pkg"
    assert "rclcpp" in [d.name for d in pkg.exec_depends]


def test_dispatch_unknown_filename_raises() -> None:
    with pytest.raises(ValueError, match="manifest"):
        parse_additional_manifest(
            "Cargo.toml", "", ros_distro="kilted", source="github.com/foo/bar"
        )


def test_missing_build_type_raises() -> None:
    bad = FIXTURES / "missing_build_type" / "pixi.toml"
    bad.parent.mkdir(parents=True, exist_ok=True)
    bad.write_text(
        '[package]\n'
        'name = "x"\n'
        'version = "0.0.1"\n'
        '[package.metadata.ros]\n'
    )
    with pytest.raises(KeyError, match="build_type"):
        parse_pixi_package(bad, ros_distro="kilted")
