"""The ROS distribution mutex package.

RoboStack ships one mutex package per ROS distribution so that only a single
distribution can be installed in an environment at a time. ``mutex_package`` in
``vinca.yaml`` accepts two shapes: a bare string, which is a literal dependency
spec on an externally maintained mutex, or a mapping, which additionally makes
vinca generate the mutex recipe itself. Only the mapping form produces a recipe;
both forms produce a dependency.
"""

from __future__ import annotations

from typing import Any

from vinca.distro import Distro
from vinca.schemas import MutexPackage, VincaConfig, VincaYaml


def parse_mutex_package_config(vinca_conf: VincaYaml) -> MutexPackage | None:
    """The dictionary-style mutex configuration, with its build number defaulting to
    the distribution's.

    String values are legacy dependency declarations and therefore do not produce a
    standalone mutex recipe.
    """
    mutex_package = vinca_conf.mutex_package
    if not isinstance(mutex_package, MutexPackage):
        return None
    if mutex_package.build_number is not None:
        return mutex_package
    build_number = vinca_conf.build_number
    return mutex_package.model_copy(
        update={"build_number": 1 if build_number is None else build_number}
    )


def get_mutex_package_dependency(vinca_conf: VincaYaml, distro: Distro) -> str | None:
    """Return the dependency spec for legacy or dictionary-style configuration."""
    mutex_package = vinca_conf.mutex_package
    if not mutex_package:
        return None
    if isinstance(mutex_package, str):
        return mutex_package

    version_parts = mutex_package.version.split(".")
    pin_depth = len(mutex_package.upper_bound.split("."))
    pin = ".".join(version_parts[:pin_depth]) + ".*"
    return f"{mutex_package.name} {pin} {distro.name}_*"


def should_skip_mutex_package(
    vinca_conf: VincaConfig, mutex_name: str, mutex_version: str
) -> bool:
    """Return whether the configured skip set already contains this mutex."""
    key = (mutex_name, mutex_version) if vinca_conf.trigger_new_versions else mutex_name
    return key in vinca_conf.skip_built_packages


def generate_mutex_package_recipe(
    vinca_conf: VincaYaml, distro: Distro
) -> dict[str, Any] | None:
    """Generate a recipe for dictionary-style mutex configuration."""
    mutex_config = parse_mutex_package_config(vinca_conf)
    if mutex_config is None:
        return None

    name = mutex_config.name
    distro_name = distro.name
    return {
        "package": {"name": name, "version": mutex_config.version},
        "build": {
            "number": mutex_config.build_number,
            "string": f"{distro_name}_{mutex_config.build_number}",
            "script": "",
        },
        "requirements": {
            **(
                {"run_constraints": mutex_config.run_constraints}
                if mutex_config.run_constraints
                else {}
            ),
            "run_exports": {
                "weak": [
                    f"${{{{ pin_subpackage('{name}', upper_bound='{mutex_config.upper_bound}') }}}}"
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
