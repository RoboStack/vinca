"""ROS distribution mutex package configuration and recipe generation."""

from __future__ import annotations

from typing import Any

_REQUIRED_FIELDS = ("name", "version", "upper_bound", "run_constraints")


def parse_mutex_package_config(vinca_conf: dict[str, Any]) -> dict[str, Any] | None:
    """Validate and normalize dictionary-style mutex configuration.

    String values are legacy dependency declarations and therefore do not produce a
    standalone mutex recipe.
    """
    mutex_package = vinca_conf.get("mutex_package")
    if not mutex_package or isinstance(mutex_package, str):
        return None
    if not isinstance(mutex_package, dict):
        raise ValueError(
            "mutex_package must be either a string or a dictionary, "
            f"got {type(mutex_package).__name__}"
        )

    missing = [field for field in _REQUIRED_FIELDS if field not in mutex_package]
    if missing:
        raise ValueError(
            f"mutex_package configuration is missing required fields: {missing}"
        )

    result = dict(mutex_package)
    result.setdefault("build_number", vinca_conf.get("build_number", 1))
    return result


def get_mutex_package_dependency(vinca_conf: dict[str, Any], distro: Any) -> str | None:
    """Return the dependency spec for legacy or dictionary-style configuration."""
    mutex_package = vinca_conf.get("mutex_package")
    if not mutex_package:
        return None
    if isinstance(mutex_package, str):
        return mutex_package

    try:
        mutex_config = parse_mutex_package_config(vinca_conf)
    except ValueError as error:
        raise ValueError(
            f"Error parsing mutex_package configuration: {error}"
        ) from error

    version_parts = mutex_config["version"].split(".")
    pin_depth = len(mutex_config["upper_bound"].split("."))
    pin = ".".join(version_parts[:pin_depth]) + ".*"
    return f"{mutex_config['name']} {pin} {distro.name}_*"


def should_skip_mutex_package(
    vinca_conf: dict[str, Any], mutex_name: str, mutex_version: str
) -> bool:
    """Return whether the configured skip set already contains this mutex."""
    skipped = vinca_conf.get("skip_built_packages", [])
    key = (
        (mutex_name, mutex_version)
        if vinca_conf.get("trigger_new_versions")
        else mutex_name
    )
    return key in skipped


def generate_mutex_package_recipe(
    vinca_conf: dict[str, Any], distro: Any
) -> dict[str, Any] | None:
    """Generate a recipe for dictionary-style mutex configuration."""
    try:
        mutex_config = parse_mutex_package_config(vinca_conf)
    except ValueError as error:
        raise ValueError(f"Cannot generate mutex package recipe: {error}") from error
    if mutex_config is None:
        return None

    name = mutex_config["name"]
    distro_name = distro.name
    return {
        "package": {"name": name, "version": mutex_config["version"]},
        "build": {
            "number": mutex_config["build_number"],
            "string": f"{distro_name}_{mutex_config['build_number']}",
            "script": "",
        },
        "requirements": {
            "run_constraints": mutex_config["run_constraints"],
            "run_exports": {
                "weak": [
                    f"${{{{ pin_subpackage('{name}', upper_bound='{mutex_config['upper_bound']}') }}}}"
                ]
            },
        },
        "about": {
            "homepage": f"https://github.com/robostack/ros-{distro_name}",
            "license": "BSD-3-Clause",
            "summary": (
                "The ROS2 distro mutex. To switch between ROS2 versions, you need "
                f"to change the mutex.\nE.g. mamba install {name}=*={distro_name} "
                f"to switch to {distro_name}."
            ),
        },
    }
