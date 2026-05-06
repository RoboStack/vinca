"""Parse pixi.toml manifests into catkin_pkg.Package objects.

This is the shim that lets vinca consume pixi.toml as if it were package.xml.
The shim returns a real catkin_pkg.package.Package so all downstream vinca
logic (recipe gen, dep resolution, etc.) is unchanged.
"""

from __future__ import annotations

import re
import tomllib
from pathlib import Path
from typing import Any, cast

from catkin_pkg.group_membership import GroupMembership
from catkin_pkg.package import (
    Dependency,
    Export,
    Package,
    Person,
    parse_package_string,
)


_AUTHOR_RE = re.compile(r"^\s*(?P<name>.+?)\s*<(?P<email>[^>]+)>\s*$")


def _parse_person(spec: str) -> Person:
    """Parse a "Name <email>" string into a catkin_pkg Person."""
    match = _AUTHOR_RE.match(spec)
    if match:
        return Person(name=match["name"], email=match["email"])
    return Person(name=spec.strip())


def _ros_pkg_name(conda_name: str, ros_distro: str) -> str:
    """Strip `ros-<distro>-` prefix and convert dashes to underscores.

    `ros-kilted-rclcpp` → `rclcpp`. Non-ROS conda names pass through untouched
    (vinca's robostack.yaml maps them to the right thing during recipe gen).
    """
    prefix = f"ros-{ros_distro}-"
    if conda_name.startswith(prefix):
        return conda_name[len(prefix) :].replace("-", "_")
    return conda_name


# Operators are matched longest-first so `>=` doesn't get parsed as `>` then `=`.
_OPS: tuple[tuple[str, str], ...] = (
    ("==", "version_eq"),
    (">=", "version_gte"),
    ("<=", "version_lte"),
    (">", "version_gt"),
    ("<", "version_lt"),
)


def _parse_version_spec(spec: str, conda_name: str) -> dict[str, str]:
    """Translate a pixi/conda version spec into catkin_pkg Dependency fields.

    Supports `==`, `>=`, `<=`, `>`, `<`, comma-separated AND clauses, and `*`
    (or empty) for unconstrained. Rejects conda-isms package.xml can't express:
    bare `=`, wildcards (`1.2.*`), OR (`|`), compatible release (`~=`).
    """
    spec = spec.strip()
    if spec in {"", "*"}:
        return {}

    out: dict[str, str] = {}
    for clause in spec.split(","):
        clause = clause.strip()
        for op, field in _OPS:
            if clause.startswith(op):
                version = clause[len(op) :].strip()
                if not version or any(c in version for c in "*|~"):
                    raise ValueError(
                        f"{conda_name}: unsupported version spec {spec!r} "
                        f"(wildcards, '|', '~=' not supported)"
                    )
                if field in out:
                    raise ValueError(
                        f"{conda_name}: duplicate {op} in version spec {spec!r}"
                    )
                out[field] = version
                break
        else:
            raise ValueError(
                f"{conda_name}: unsupported version spec {spec!r} "
                "(only ==, >=, <=, >, < and comma-AND are supported)"
            )
    return out


def _make_dep(conda_name: str, version_spec: str, ros_distro: str) -> Dependency:
    """Build a Dependency, with `evaluated_condition=True` so vinca keeps it."""
    dep = Dependency(
        name=_ros_pkg_name(conda_name, ros_distro),
        **_parse_version_spec(version_spec, conda_name),
    )
    dep.evaluated_condition = True
    return dep


def _require_str(value: Any, field: str, source: str) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{source}: [package].{field} must be a string, got {type(value).__name__}")
    return value


def _require_str_list(value: Any, field: str, source: str) -> list[str]:
    if not isinstance(value, list) or not all(isinstance(x, str) for x in value):
        raise TypeError(f"{source}: {field} must be a list of strings")
    return cast(list[str], value)


def _require_str_dict(value: Any, field: str, source: str) -> dict[str, str]:
    if not isinstance(value, dict) or not all(
        isinstance(k, str) and isinstance(v, str) for k, v in value.items()
    ):
        raise TypeError(f"{source}: {field} must be a {{str: str}} table")
    return cast(dict[str, str], value)


def parse_pixi_package(path: str | Path, ros_distro: str) -> Package:
    """Parse a pixi.toml file at *path* into a catkin_pkg Package."""
    path = Path(path)
    return parse_pixi_package_string(
        path.read_text(), ros_distro=ros_distro, source=str(path), filename=str(path)
    )


def parse_additional_manifest(
    filename: str,
    content: str,
    ros_distro: str,
    *,
    source: str,
) -> Package:
    """Parse a fetched manifest by filename into a catkin_pkg Package.

    Used for `rosdistro_additional_recipes.yaml` entries — the file is
    fetched (e.g. from GitHub) and dispatched here based on filename so the
    caller doesn't have to know which parser to use.

    Args:
        filename: The manifest filename (e.g. "pixi.toml" or "package.xml").
        content: Raw file contents.
        ros_distro: ROS distro name, forwarded to the pixi shim.
        source: Human-readable label used in error messages.
    """
    if filename.endswith("pixi.toml"):
        return parse_pixi_package_string(
            content, ros_distro=ros_distro, source=source
        )
    if filename.endswith("package.xml"):
        return cast(Package, parse_package_string(content))
    raise ValueError(
        f"{source}: unsupported manifest filename {filename!r} "
        "(expected 'pixi.toml' or 'package.xml')"
    )


def parse_pixi_package_string(
    content: str,
    ros_distro: str,
    *,
    source: str,
    filename: str | None = None,
) -> Package:
    """Parse pixi.toml *content* into an equivalent catkin_pkg Package.

    Args:
        content: Raw TOML text for a pixi.toml.
        ros_distro: ROS distro name (e.g. "kilted") used to translate
            `ros-<distro>-foo` conda dep names back to bare ROS names.
        source: Human-readable label used in error messages (e.g. a path,
            URL, or "github.com/foo/bar@ref").
        filename: Optional path to set on the returned Package. Defaults to
            None when the content didn't come from disk.

    Returns:
        A catkin_pkg.package.Package populated as if parsed from a package.xml.
    """
    raw: dict[str, Any] = tomllib.loads(content)
    pkg_raw = raw.get("package")
    if not isinstance(pkg_raw, dict):
        raise KeyError(f"{source}: missing [package] table")

    name = _require_str(pkg_raw.get("name"), "name", source)
    version = _require_str(pkg_raw.get("version"), "version", source)
    description = _require_str(pkg_raw.get("description", ""), "description", source)
    license_ = _require_str(pkg_raw.get("license", ""), "license", source)
    authors_raw = _require_str_list(pkg_raw.get("authors", []), "[package].authors", source)

    metadata = pkg_raw.get("metadata", {})
    ros_meta_raw = metadata.get("ros", {}) if isinstance(metadata, dict) else {}
    if not isinstance(ros_meta_raw, dict):
        raise KeyError(f"{source}: [package.metadata.ros] must be a table")

    if "build_type" not in ros_meta_raw:
        raise KeyError(
            f"{source}: [package.metadata.ros].build_type is required "
            "(e.g. 'ament_cmake', 'ament_python', 'ament_cargo')"
        )
    build_type = _require_str(
        ros_meta_raw["build_type"], "metadata.ros.build_type", source
    )

    test = _require_str_dict(
        ros_meta_raw.get("test_dependencies", {}),
        "[package.metadata.ros.test_dependencies]",
        source,
    )
    is_message_package = bool(ros_meta_raw.get("is_message_package", False))

    host = _require_str_dict(
        pkg_raw.get("host-dependencies", {}), "[package.host-dependencies]", source
    )
    run = _require_str_dict(
        pkg_raw.get("run-dependencies", {}), "[package.run-dependencies]", source
    )

    people = [_parse_person(a) for a in authors_raw]

    build_type_export = Export(tagname="build_type")
    build_type_export.content = build_type

    build_depends = [_make_dep(n, v, ros_distro) for n, v in host.items()]
    exec_depends = [_make_dep(n, v, ros_distro) for n, v in run.items()]
    test_depends = [_make_dep(n, v, ros_distro) for n, v in test.items()]
    buildtool_depends: list[Dependency] = []
    member_of_groups: list[GroupMembership] = []

    if is_message_package:
        buildtool_depends.append(
            _make_dep("rosidl_default_generators", "*", ros_distro)
        )
        membership = GroupMembership(name="rosidl_interface_packages")
        membership.evaluated_condition = True
        member_of_groups.append(membership)

    return Package(
        package_format=3,
        name=name,
        version=version,
        description=description,
        licenses=[license_],
        authors=people,
        maintainers=people,
        build_depends=build_depends,
        buildtool_depends=buildtool_depends,
        exec_depends=exec_depends,
        test_depends=test_depends,
        member_of_groups=member_of_groups,
        exports=[build_type_export],
        filename=filename,
    )
