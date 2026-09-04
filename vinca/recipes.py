"""Translation of ROS package metadata into rattler-build recipe *outputs*.

An "output" is the ``package``/``build``/``requirements``/``about`` mapping for a
single conda package. This module owns dependency resolution and the
cross-compilation fixups applied to the requirement sections; it does not read or
write files. Its siblings are :mod:`vinca.sources` (the ``source`` section) and
:mod:`vinca.template` (rendering outputs to disk).

Two ownership rules matter for callers:

* ``vinca_conf`` is treated as read-only. Nothing here mutates it.
* The ``unsatisfied`` set passed to :func:`generate_output` is owned by the caller
  and is mutated in place with every dependency that could not be resolved, so a
  single set can accumulate across a whole generation run.
"""

from __future__ import annotations

import copy
import os
from typing import Any

import catkin_pkg

from vinca.distro import Distro
from vinca.license_utils import convert_to_spdx_license
from vinca.mutex import get_mutex_package_dependency
from vinca.naming import get_package_prefix, normalize_package_dependency
from vinca.resolve import resolve_pkgname
from vinca.utils import (
    get_pkg_additional_info,
    get_pkg_build_number,
    is_dummy_metapackage,
)

Requirements = dict[str, list[Any]]

_BUILD_SCRIPTS = {
    "cmake": "${{ '$RECIPE_DIR/build_catkin.sh' if unix or wasm32 else '%RECIPE_DIR%\\\\bld_catkin.bat' }}",
    "catkin": "${{ '$RECIPE_DIR/build_catkin.sh' if unix or wasm32 else '%RECIPE_DIR%\\\\bld_catkin.bat' }}",
    "ament_cmake": "${{ '$RECIPE_DIR/build_ament_cmake.sh' if unix or wasm32 else '%RECIPE_DIR%\\\\bld_ament_cmake.bat' }}",
    "ament_python": "${{ '$RECIPE_DIR/build_ament_python.sh' if unix or wasm32 else '%RECIPE_DIR%\\\\bld_ament_python.bat' }}",
}

_BASE_REQUIREMENTS = {
    "build": [
        "${{ compiler('cxx') }}",
        "${{ compiler('c') }}",
        {
            "if": "target_platform!='emscripten-wasm32'",
            "then": ["${{ stdlib('c') }}"],
        },
        "ninja",
        "python",
        "setuptools",
        "git",
        "git-lfs",
        {"if": "unix", "then": ["patch", "make", "coreutils"]},
        {"if": "win", "then": ["m2-patch"]},
        {"if": "osx", "then": ["tapi"]},
        {"if": "build_platform != target_platform", "then": ["pkg-config"]},
        "cmake",
        "cython",
        {
            "if": "build_platform != target_platform",
            "then": ["python", "cross-python_${{ target_platform }}", "numpy"],
        },
    ],
    "host": [
        {"if": "build_platform == target_platform", "then": ["pkg-config"]},
        "python",
        "numpy",
        "pip",
    ],
    "run": [],
}


def get_depmods(
    vinca_conf: dict[str, Any], package_name: str, distro: Distro
) -> tuple[Requirements, Requirements]:
    """Return normalized dependency removals and additions by requirement section."""
    configured = vinca_conf.get("depmods", {}).get(package_name, {})
    removed = {section: [] for section in ("build", "host", "run")}
    added = {section: [] for section in ("build", "host", "run")}
    for section in removed:
        removed[section] = [
            normalize_package_dependency(dependency, distro, vinca_conf)
            for dependency in configured.get(f"remove_{section}", [])
        ]
        added[section] = [
            normalize_package_dependency(dependency, distro, vinca_conf)
            for dependency in configured.get(f"add_{section}", [])
        ]
    return removed, added


def _dummy_constraint(
    config: dict[str, Any], version: str, shortname: str
) -> tuple[str, str]:
    dependency = config.get("dep_name")
    if not dependency:
        raise RuntimeError(f"Missing 'dep_name' for dummy recipe of {shortname}")
    upper_bound = config.get("upper_bound") or config.get("max_pin")
    if not upper_bound:
        raise RuntimeError(
            f"Missing 'upper_bound' or 'max_pin' for dummy recipe of {shortname}"
        )

    package_version = config.get("override_version", version)
    parts = [int(part) for part in package_version.split(".")]
    pin_depth = len(upper_bound.split("."))
    upper_parts = parts[:pin_depth]
    upper_parts[-1] += 1
    upper_parts += [0] * (len(parts) - pin_depth)
    upper = ".".join(map(str, upper_parts)) + "a0"
    return package_version, f"{dependency} >={package_version}, <{upper}"


def _initial_output(
    shortname: str,
    package_name: str,
    version: str,
    vinca_conf: dict[str, Any],
) -> dict[str, Any]:
    if not is_dummy_metapackage(shortname, vinca_conf):
        return {
            "package": {"name": package_name, "version": version},
            "requirements": copy.deepcopy(_BASE_REQUIREMENTS),
            "build": {"script": ""},
        }

    dummy_config = get_pkg_additional_info(shortname, vinca_conf)[
        "generate_dummy_package_with_run_deps"
    ]
    package_version, constraint = _dummy_constraint(dummy_config, version, shortname)
    return {
        "package": {"name": package_name, "version": package_version},
        "build": {
            "number": get_pkg_build_number(
                vinca_conf.get("build_number", 0), package_name, vinca_conf
            ),
            "script": "",
        },
        "requirements": {"build": [], "host": [], "run": [constraint]},
    }


def _evaluated_dependency_names(dependencies) -> list[str]:
    """Return the names of the catkin dependencies whose conditions evaluated true."""
    return [
        dependency.name for dependency in dependencies if dependency.evaluated_condition
    ]


def _resolve_into(
    dependencies: list[str],
    destination: list[Any],
    vinca_conf: dict[str, Any],
    distro: Distro,
    unsatisfied: set[str],
    *,
    runtime: bool = False,
) -> None:
    """Resolve ROS names to conda names, recording anything that cannot be mapped."""
    for dependency in dependencies:
        resolved = resolve_pkgname(dependency, vinca_conf, distro, is_rundep=runtime)
        if resolved:
            destination.extend(resolved)
        else:
            unsatisfied.add(dependency)


def _apply_depmods(
    requirements: Requirements, removed: Requirements, added: Requirements
) -> None:
    """Apply configured ``dependencies.yaml`` overrides; additions are applied first."""
    for section in requirements:
        requirements[section].extend(added[section])
        requirements[section] = [
            dependency
            for dependency in requirements[section]
            if dependency not in removed[section]
        ]


def _replace_with_selector(
    requirements: list[Any],
    dependency: str,
    condition: str,
    *,
    destination: list[Any] | None = None,
) -> None:
    """Swap a plain dependency for a conditional one, optionally moving it elsewhere.

    ``destination`` lets a dependency be lifted out of one requirement section and
    re-added to another, which is how host-only tools are moved into ``build`` for
    cross-compilation.
    """
    if dependency not in requirements:
        return
    while dependency in requirements:
        requirements.remove(dependency)
    (destination if destination is not None else requirements).append(
        {"if": condition, "then": [dependency]}
    )


def _adjust_requirements(requirements: Requirements, package_prefix: str) -> None:
    """Apply platform and cross-compilation fixups, then sort and deduplicate.

    These rules are empirical: they encode which dependencies break when building
    for Emscripten or when the build and target platforms differ.
    """
    # For Emscripten, cmake is only needed while building, never at run time.
    _replace_with_selector(
        requirements["run"], "cmake", "target_platform != 'emscripten-wasm32'"
    )
    if "cmake" in requirements["host"]:
        requirements["host"].remove("cmake")
        if "cmake" not in requirements["build"]:
            requirements["build"].append("cmake")

    mimick_vendor = f"{package_prefix}-mimick-vendor"
    _replace_with_selector(
        requirements["build"], mimick_vendor, "target_platform != 'emscripten-wasm32'"
    )
    _replace_with_selector(
        requirements["host"],
        mimick_vendor,
        "target_platform != 'emscripten-wasm32'",
        destination=requirements["build"],
    )

    # Emscripten resolves the generators out of `build` rather than `host`.
    rosidl_generators = f"{package_prefix}-rosidl-default-generators"
    if rosidl_generators in requirements["host"]:
        requirements["build"].append(
            {
                "if": "target_platform == 'emscripten-wasm32'",
                "then": [rosidl_generators],
            }
        )

    requirements["run"].sort(key=_requirement_sort_key)
    requirements["host"].sort(key=_requirement_sort_key)

    if f"{package_prefix}-pybind11-vendor" in requirements["host"]:
        requirements["host"].append("pybind11")
    for dependency in ("pybind11", "qt-main"):
        if dependency in requirements["host"]:
            requirements["build"].append(
                {"if": "build_platform != target_platform", "then": [dependency]}
            )

    # These run on the build machine, so a target-platform build of them is useless:
    # keep them in `host` only when not cross-compiling, and in `build` when we are.
    for dependency in ("pyqt-builder", "git", "doxygen", "git-lfs"):
        _replace_with_selector(
            requirements["host"],
            dependency,
            "build_platform == target_platform",
        )
        if any(
            item == {"if": "build_platform == target_platform", "then": [dependency]}
            for item in requirements["host"]
        ):
            requirements["build"].append(
                {"if": "build_platform != target_platform", "then": [dependency]}
            )

    for section, dependencies in requirements.items():
        unique = []
        for dependency in dependencies:
            if dependency not in unique:
                unique.append(dependency)
        requirements[section] = unique


def _requirement_sort_key(requirement):
    """Sort conditional requirements by their condition, plain ones by their name."""
    return (
        next(iter(requirement.values()))
        if isinstance(requirement, dict)
        else requirement
    )


def _add_metadata(
    output: dict[str, Any], package: Any, shortname: str, distro: Distro
) -> None:
    """Populate ``about`` from package.xml and rosdistro metadata."""
    about = output["about"] = {}
    for url in package.urls:
        if url.type == "website":
            about["homepage"] = url.url
    repository = distro.get_repository_url(shortname, package.urls)
    if repository:
        about["repository"] = repository
    if package.licenses:
        license_expression = convert_to_spdx_license(
            [str(license) for license in package.licenses], package_name=shortname
        )
        if license_expression:
            about["license"] = license_expression
    if package.description:
        about["summary"] = package.description


def generate_output(
    shortname: str,
    vinca_conf: dict[str, Any],
    distro: Distro,
    version: str,
    all_packages: list[Any] | None = None,
    unsatisfied: set[str] | None = None,
    *,
    dependencies_only: bool = False,
) -> dict[str, Any] | Requirements | None:
    """Generate one package output from ROS metadata and vinca configuration.

    ``all_packages`` supplies the parsed manifests used to expand ``<group_depend>``
    entries. Unresolvable dependency names are added to ``unsatisfied``, which the
    caller owns.

    Returns ``None`` when the package is not selected, cannot be named, has no
    release manifest, or uses an unsupported build type. When ``dependencies_only``
    is set the return value is the ``requirements`` mapping rather than a complete
    output, which lets the pinning code collect dependencies without paying for
    license and metadata lookups.
    """
    all_packages = all_packages or []
    unsatisfied = unsatisfied if unsatisfied is not None else set()
    if shortname not in vinca_conf["_selected_pkgs"]:
        return None

    package_names = resolve_pkgname(shortname, vinca_conf, distro)
    if not package_names:
        return None
    output = _initial_output(shortname, package_names[0], version, vinca_conf)

    xml = distro.get_release_package_xml(shortname)
    if not xml:
        # Without a snapshot, rosdistro can return no manifest for an unreleased package.
        print(f"Skip {shortname} because no release package.xml is available.")
        return None
    package = catkin_pkg.package.parse_package_string(xml)
    package.evaluate_conditions(os.environ)

    python_dependencies = resolve_pkgname("python", vinca_conf, distro)
    output["requirements"]["run"].extend(python_dependencies)
    output["requirements"]["host"].extend(python_dependencies)

    is_dummy = is_dummy_metapackage(shortname, vinca_conf)
    build_type = package.get_build_type()
    if not is_dummy:
        try:
            output["build"]["script"] = _BUILD_SCRIPTS[build_type]
        except KeyError:
            print(f"Unknown build type for {shortname}: {build_type}")
            return None
        if build_type == "ament_python":
            output["requirements"]["host"].extend(
                resolve_pkgname("python-setuptools", vinca_conf, distro)
            )
        if build_type in ("cmake", "catkin", "ament_cmake"):
            output["requirements"]["build"].append(
                {
                    "if": "osx",
                    "then": [
                        "clang-tools ${{ (cxx_compiler_version ~ '.*') if "
                        "cxx_compiler_version is defined else '*' }}"
                    ],
                }
            )

    mutex_dependency = get_mutex_package_dependency(vinca_conf, distro)
    if mutex_dependency:
        output["requirements"]["host"].append(mutex_dependency)
        output["requirements"]["run"].append(mutex_dependency)

    package_prefix = get_package_prefix(distro, vinca_conf)
    if not distro.check_ros1() and shortname not in {
        "ament_cmake_core",
        "ament_package",
        "ros_workspace",
        "ros_environment",
    }:
        output["requirements"]["host"].extend(
            [f"{package_prefix}-ros-environment", f"{package_prefix}-ros-workspace"]
        )
        output["requirements"]["run"].append(f"{package_prefix}-ros-workspace")

    group_dependencies = []
    for dependency in package.group_depends:
        dependency.extract_group_members(all_packages)
        group_dependencies.extend(dependency.members)

    build_tools = _evaluated_dependency_names(
        [*package.buildtool_depends, *package.buildtool_export_depends]
    )
    build_dependencies = (
        _evaluated_dependency_names(
            [
                *package.build_depends,
                *package.build_export_depends,
                *package.test_depends,
            ]
        )
        + group_dependencies
    )
    # Build tools normally belong in `host`, but git has to be in `build` so that it
    # is runnable on the build machine when cross-compiling. cmake is already part of
    # the base build requirements, so re-adding it would only create a duplicate.
    for dependency in build_tools:
        resolved = resolve_pkgname(dependency, vinca_conf, distro)
        if not resolved:
            unsatisfied.add(dependency)
        elif "git" in resolved:
            output["requirements"]["build"].extend(resolved)
        elif dependency != "cmake":
            build_dependencies.append(dependency)

    # cyclonedds generates code during the build, so a build-platform copy is needed.
    if shortname == "cyclonedds" or "cyclonedds" in build_dependencies + build_tools:
        output["requirements"]["build"].append(
            {
                "if": "build_platform != target_platform",
                "then": [f"{package_prefix}-cyclonedds"],
            }
        )

    _resolve_into(
        build_dependencies,
        output["requirements"]["host"],
        vinca_conf,
        distro,
        unsatisfied,
    )
    run_dependencies = (
        _evaluated_dependency_names(
            [
                *package.run_depends,
                *package.exec_depends,
                *package.build_export_depends,
                *package.buildtool_export_depends,
            ]
        )
        + group_dependencies
    )
    _resolve_into(
        run_dependencies,
        output["requirements"]["run"],
        vinca_conf,
        distro,
        unsatisfied,
        runtime=True,
    )

    removed, added = get_depmods(vinca_conf, package.name, distro)
    _apply_depmods(output["requirements"], removed, added)
    _adjust_requirements(output["requirements"], package_prefix)
    if dependencies_only:
        return output["requirements"]
    _add_metadata(output, package, shortname, distro)
    return output
