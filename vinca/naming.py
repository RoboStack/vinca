"""Helpers for selecting and generating ROS package names."""

from __future__ import annotations

from typing import Any, Optional

from vinca.distro import Distro
from vinca.mutex import get_mutex_package_dependency
from vinca.schemas import PackageNameMode as PackageNameMode
from vinca.schemas import VincaYaml


def get_package_prefix(distro: Distro, vinca_conf: VincaYaml) -> str:
    """Return the prefix used for generated packages and their dependencies."""
    if vinca_conf.package_name_mode is PackageNameMode.LEGACY:
        return distro.get_legacy_package_prefix()
    return distro.get_package_prefix()


def get_new_package_name(pkg_shortname: str, distro: Distro) -> str:
    """Return the ROS-major-version package name."""
    return f"{distro.get_package_prefix()}-{pkg_shortname.replace('_', '-')}"


def get_legacy_package_name(pkg_shortname: str, distro: Distro) -> str:
    """Return the distro-qualified package name."""
    return f"{distro.get_legacy_package_prefix()}-{pkg_shortname.replace('_', '-')}"


def get_package_name(pkg_shortname: str, distro: Distro, vinca_conf: VincaYaml) -> str:
    """Return the primary generated name for a ROS package."""
    if vinca_conf.package_name_mode is PackageNameMode.LEGACY:
        return get_legacy_package_name(pkg_shortname, distro)
    return get_new_package_name(pkg_shortname, distro)


def normalize_package_dependency(
    dependency: Any, distro: Distro, vinca_conf: VincaYaml
) -> Any:
    """Rewrite current-distro legacy dependency names for the selected mode."""
    if isinstance(dependency, dict):
        return {
            key: normalize_package_dependency(value, distro, vinca_conf)
            for key, value in dependency.items()
        }
    if isinstance(dependency, list):
        return [
            normalize_package_dependency(value, distro, vinca_conf)
            for value in dependency
        ]
    if not isinstance(dependency, str):
        return dependency

    legacy_prefix = f"{distro.get_legacy_package_prefix()}-"
    if not dependency.startswith(legacy_prefix):
        return dependency

    package_prefix = f"{get_package_prefix(distro, vinca_conf)}-"
    return package_prefix + dependency[len(legacy_prefix) :]


def generate_legacy_compatibility_output(
    output: dict[str, Any], pkg_shortname: str, distro: Distro, vinca_conf: VincaYaml
) -> Optional[dict[str, Any]]:
    """Create an old-name package that depends on the renamed package, if requested."""
    if vinca_conf.package_name_mode is not PackageNameMode.BOTH:
        return None

    canonical_name = get_new_package_name(pkg_shortname, distro)
    if output["package"]["name"] != canonical_name:
        return None

    legacy_name = get_legacy_package_name(pkg_shortname, distro)
    version = output["package"]["version"]
    run = [f"{canonical_name} =={version}"]
    # Several distributions can publish the same ros2-<pkg> version to one channel;
    # requiring this distribution's mutex (as the canonical packages do) keeps the
    # alias from being satisfied by another distribution's build.
    mutex_dependency = get_mutex_package_dependency(vinca_conf, distro)
    if mutex_dependency:
        run.append(mutex_dependency)
    return {
        "package": {"name": legacy_name, "version": version},
        "build": {"script": ""},
        "requirements": {"run": run},
        "about": {"summary": f"Compatibility package for {canonical_name}"},
    }


def is_legacy_compatibility_output(
    output: dict[str, Any], distro: Distro, vinca_conf: VincaYaml
) -> bool:
    """Return whether an output is one of the generated compatibility packages."""
    if vinca_conf.package_name_mode is not PackageNameMode.BOTH:
        return False

    legacy_prefix_getter = getattr(distro, "get_legacy_package_prefix", None)
    if legacy_prefix_getter is None:
        return False

    name = output["package"]["name"]
    legacy_prefix = f"{legacy_prefix_getter()}-"
    if not name.startswith(legacy_prefix):
        return False

    shortname = name[len(legacy_prefix) :]
    canonical_name = get_new_package_name(shortname, distro)
    version = output["package"]["version"]
    # the canonical package comes first, optionally followed by the mutex
    run = output.get("requirements", {}).get("run") or []
    return run[:1] == [f"{canonical_name} =={version}"]
