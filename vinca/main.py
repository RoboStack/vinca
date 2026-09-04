#!/usr/bin/env python

from __future__ import annotations

import argparse
import glob
import os
import sys
from typing import Any, Optional

import catkin_pkg
import ruamel.yaml

from vinca import __version__, config
from vinca.utils import get_pkg_build_number, get_repodata

from .configuration import read_snapshot as read_snapshot
from .configuration import read_vinca_yaml as load_vinca_yaml
from .distro import Distro
from .mutex import (
    generate_mutex_package_recipe as generate_mutex_package_recipe,
)
from .mutex import (
    get_mutex_package_dependency as get_mutex_package_dependency,
)
from .mutex import (
    parse_mutex_package_config as parse_mutex_package_config,
)
from .mutex import (
    should_skip_mutex_package as should_skip_mutex_package,
)
from .naming import (
    generate_legacy_compatibility_output,
    get_package_name,
    is_legacy_compatibility_output,
)
from .platforms import get_conda_subdir as detect_conda_subdir
from .recipes import generate_output as build_output
from .recipes import get_depmods as get_depmods
from .resolve import resolve_pkgname
from .sources import (
    generate_fat_source as build_fat_source,
)
from .sources import (
    generate_source as build_source,
)
from .sources import (
    generate_source_version as build_source_version,
)
from .sources import (
    source_reference,
)
from .template import write_recipe, write_recipe_package

unsatisfied_deps = set()
distro = None


def ensure_list(obj):
    if not obj:
        return []
    assert isinstance(obj, list)
    return obj


def get_conda_subdir():
    """Return the configured target platform, or detect the host platform."""
    selected = getattr(config.parsed_args, "platform", None)
    return detect_conda_subdir(selected)


def parse_command_line(argv):
    """
    Parse command line argument. See -h option.
    :param argv: the actual program arguments
    :return: parsed arguments
    """
    import textwrap

    default_dir = "."

    example = textwrap.dedent(
        """
      Examples:
        {0} -d ./examples/
      See: https://github.com/RoboStack/vinca
    """
    ).format(os.path.basename(argv[0]))
    formatter_class = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(
        description="Conda recipe generator for ROS packages",
        epilog=example,
        formatter_class=formatter_class,
    )
    parser.add_argument(
        "-V", "--version", action="version", version="%(prog)s {}".format(__version__)
    )
    parser.add_argument(
        "-d",
        "--dir",
        dest="dir",
        default=default_dir,
        help="The directory to process (default: {}).".format(default_dir),
    )
    parser.add_argument(
        "-s",
        "--skip",
        dest="skip_already_built_repodata",
        default=[],
        help="Skip already built from repodata.",
    )
    parser.add_argument(
        "-m",
        "--multiple",
        dest="multiple_file",
        action="store_const",
        const=True,
        default=False,
        help="Create one recipe for package.",
    )
    parser.add_argument(
        "-n",
        "--trigger-new-versions",
        dest="trigger_new_versions",
        action="store_const",
        const=True,
        default=False,
        help="Trigger the build of packages that have new versions available.",
    )
    parser.add_argument(
        "--source",
        dest="source",
        action="store_const",
        const=True,
        default=False,
        help="Create recipe with develop repo.",
    )
    parser.add_argument(
        "-p", "--package", dest="package", default=None, help="The package.xml path."
    )
    parser.add_argument(
        "--platform",
        dest="platform",
        default=None,
        help="The conda platform to check existing recipes for.",
    )
    arguments = parser.parse_args(argv[1:])
    config.parsed_args = arguments
    config.selected_platform = get_conda_subdir()
    return arguments


def read_vinca_yaml(filepath):
    """Load configuration for the currently selected conda platform."""
    return load_vinca_yaml(filepath, target_platform=get_conda_subdir())


def should_skip_output(output, vinca_conf):
    """Return whether an individual output is already available."""
    name = output["package"]["name"]
    version = output["package"]["version"]
    skip_built_packages = vinca_conf.get("skip_built_packages", [])
    if vinca_conf.get("trigger_new_versions"):
        return (name, version) in skip_built_packages
    return name in skip_built_packages


def append_output_with_compatibility(
    outputs, output, pkg_shortname, distro, vinca_conf
):
    """Append canonical and compatibility outputs, filtering each independently."""
    candidate_outputs = [output]
    compatibility_output = generate_legacy_compatibility_output(
        output, pkg_shortname, distro, vinca_conf
    )
    if compatibility_output is not None:
        candidate_outputs.append(compatibility_output)

    for candidate in candidate_outputs:
        if should_skip_output(candidate, vinca_conf):
            print(f"Skipping {candidate['package']['name']} (already built)")
        else:
            outputs.append(candidate)


def generate_output(
    pkg_shortname: str,
    vinca_conf: dict[str, Any],
    distro: Distro,
    version: str,
    all_pkgs: Optional[list[Any]] = None,
    *,
    dependencies_only: bool = False,
) -> Optional[dict[str, Any]]:
    """Backward-compatible entry point bound to this module's unsatisfied-dep set."""
    return build_output(
        pkg_shortname,
        vinca_conf,
        distro,
        version,
        all_packages=all_pkgs,
        unsatisfied=unsatisfied_deps,
        dependencies_only=dependencies_only,
    )


def get_group_dependency_packages(distro: Distro) -> list[Any]:
    """Parse the packages needed to expand package.xml group dependencies."""

    packages = []
    for package_name in distro.get_depends("ros_base"):
        xml = distro.get_release_package_xml(package_name)
        if not xml:
            continue
        package = catkin_pkg.package.parse_package_string(xml)
        package.evaluate_conditions(os.environ)
        packages.append(package)
    return packages


def generate_dependency_requirements(
    distro: Distro, vinca_conf: dict[str, Any], all_pkgs: list[Any]
) -> list[dict[str, Any]]:
    """Collect requirement mappings without generating complete recipe outputs."""

    requirements = []
    for pkg_shortname in vinca_conf["_selected_pkgs"]:
        if not distro.check_package(pkg_shortname):
            continue
        requirement = generate_output(
            pkg_shortname,
            vinca_conf,
            distro,
            distro.get_version(pkg_shortname),
            all_pkgs,
            dependencies_only=True,
        )
        if requirement is not None:
            requirements.append(requirement)

    mutex_config = parse_mutex_package_config(vinca_conf)
    if mutex_config is not None:
        requirements.append(
            {"run_constraints": mutex_config.get("run_constraints", [])}
        )
    return requirements


def generate_outputs(distro, vinca_conf):
    outputs = []

    def get_pkg(pkg_name):
        pkg = catkin_pkg.package.parse_package_string(
            distro.get_release_package_xml(pkg_name)
        )
        pkg.evaluate_conditions(os.environ)
        return pkg

    all_pkgs = [get_pkg(pkg) for pkg in distro.get_depends("ros_base")]

    for pkg_shortname in vinca_conf["_selected_pkgs"]:
        if not distro.check_package(pkg_shortname):
            print(f"Could not generate output for {pkg_shortname}")
            continue

        output: dict[str, Any] | None = None

        try:
            output = generate_output(
                pkg_shortname,
                vinca_conf,
                distro,
                distro.get_version(pkg_shortname),
                all_pkgs,
            )
        except AttributeError:
            print("Skip " + pkg_shortname + " due to invalid version / XML.")
        if output is not None:
            append_output_with_compatibility(
                outputs, output, pkg_shortname, distro, vinca_conf
            )

    # Generate mutex package if configured as dictionary
    mutex_recipe = generate_mutex_package_recipe(vinca_conf, distro)
    if mutex_recipe:
        # Check if mutex package should be skipped
        mutex_name = mutex_recipe["package"]["name"]
        mutex_version = mutex_recipe["package"]["version"]

        if should_skip_mutex_package(vinca_conf, mutex_name, mutex_version):
            print(f"Skipping mutex package {mutex_name} (already built)")
        else:
            print(f"Generating mutex package: {mutex_name}")
            outputs.append(mutex_recipe)

    return outputs


def generate_outputs_version(distro, vinca_conf):
    outputs = []
    for pkg_shortname in vinca_conf["_selected_pkgs"]:
        if not distro.check_package(pkg_shortname):
            print(f"Could not generate output for {pkg_shortname}")
            continue

        version = distro.get_version(pkg_shortname)
        output = generate_output(pkg_shortname, vinca_conf, distro, version)
        if output is not None:
            append_output_with_compatibility(
                outputs, output, pkg_shortname, distro, vinca_conf
            )

    return outputs


def _source_reference(*, url, ref, ref_type):
    """Backward-compatible entry point for source reference generation."""
    return source_reference(url=url, ref=ref, ref_type=ref_type)


def generate_source(distro, vinca_conf):
    """Backward-compatible entry point that defaults to the selected platform."""
    return build_source(distro, vinca_conf, get_conda_subdir())


def generate_source_version(distro, vinca_conf):
    """Backward-compatible entry point that defaults to the selected platform."""
    return build_source_version(distro, vinca_conf, get_conda_subdir())


def generate_fat_source(distro, vinca_conf):
    """Backward-compatible entry point for fat-recipe source generation."""
    return build_fat_source(distro, vinca_conf)


def get_selected_packages(distro, vinca_conf):
    selected_packages = set()
    skipped_packages = set()

    # Provenance tracking: record *why* each package ended up being selected.
    #   requested_by_config: package was explicitly listed in the config
    #                        (packages_select_by_deps / build_all / additional recipes)
    #   required_by:         map of package -> set of config-requested packages whose
    #                        (transitive) dependency closure pulled this package in
    requested_by_config = set()
    required_by = {}

    if vinca_conf.get("build_all", False):
        selected_packages = set(distro._distro.release_packages.keys())
        requested_by_config |= selected_packages
        # Add packages from rosdistro_additional_recipes.yaml when build_all is True
        if (
            "_additional_packages_snapshot" in vinca_conf
            and vinca_conf["_additional_packages_snapshot"]
        ):
            additional_packages = set(
                vinca_conf["_additional_packages_snapshot"].keys()
            )
            selected_packages = selected_packages.union(additional_packages)
            requested_by_config |= additional_packages
    elif vinca_conf["packages_select_by_deps"]:
        if (
            "packages_skip_by_deps" in vinca_conf
            and vinca_conf["packages_skip_by_deps"] is not None
        ):
            for i in vinca_conf["packages_skip_by_deps"]:
                print(f"Calling replace on {i}.")
                skipped_packages = skipped_packages.union([i, i.replace("-", "_")])
        print("Skipped pkgs: ", skipped_packages)
        for i in vinca_conf["packages_select_by_deps"]:
            i = i.replace("-", "_")
            selected_packages = selected_packages.union([i])
            requested_by_config.add(i)
            if i in skipped_packages:
                continue
            try:
                pkgs = distro.get_depends(i, ignore_pkgs=skipped_packages)
            except KeyError as err:
                print(f"KeyError: {err} for package {i}. Skipping.")
                # handle (rare) package names that use "-" as separator
                pkgs = distro.get_depends(i.replace("_", "-"))
                selected_packages.remove(i)
                selected_packages.add(i.replace("_", "-"))
                requested_by_config.discard(i)
                i = i.replace("_", "-")
                requested_by_config.add(i)
            selected_packages = selected_packages.union(pkgs)
            # record that the (config-requested) package `i` depends on each `dep`
            for dep in pkgs:
                required_by.setdefault(dep, set()).add(i)

    # Automatically include ros_workspace and ros_environment for ROS2 distributions
    # if any ROS2 packages are selected (these are added as dependencies automatically)
    if not distro.check_ros1() and selected_packages:
        # Check if we have any ROS packages selected (excluding the workspace/environment packages themselves)
        has_ros_packages = any(
            pkg
            not in [
                "ros_workspace",
                "ros_environment",
                "ament_cmake_core",
                "ament_package",
            ]
            for pkg in selected_packages
        )
        if has_ros_packages:
            if distro.check_package("ros_workspace"):
                selected_packages.add("ros_workspace")
                required_by.setdefault("ros_workspace", set()).add(
                    "(automatic ROS2 dependency)"
                )
            if distro.check_package("ros_environment"):
                selected_packages.add("ros_environment")
                required_by.setdefault("ros_environment", set()).add(
                    "(automatic ROS2 dependency)"
                )

    # expose provenance so a summary of *why* each recipe was generated can be printed
    vinca_conf["_pkg_provenance"] = {
        "requested_by_config": requested_by_config,
        "required_by": required_by,
    }

    result = sorted(list(selected_packages))
    return result


def parse_package(pkg, distro, vinca_conf, path):
    name = pkg["name"].replace("_", "-")
    final_name = get_package_name(name, distro, vinca_conf)

    recipe = {
        "package": {"name": final_name, "version": pkg["version"]},
        "about": {
            "homepage": "https://www.ros.org/",
            "license": [str(lic) for lic in pkg["licenses"]],
            "summary": pkg["description"],
            "maintainers": [],
        },
        "extra": {"recipe-maintainers": ["robostack"]},
        "build": {
            "number": 0,
            "script": "${{ '$RECIPE_DIR/build_catkin.sh' if unix or wasm32 else '%RECIPE_DIR%\\\\bld_catkin.bat' }}",
        },
        "source": {},
        "requirements": {
            "build": [
                "${{ compiler('cxx') }}",
                "${{ compiler('c') }}",
                {
                    "if": "target_platform!='emscripten-wasm32'",
                    "then": ["${{ stdlib('c') }}"],
                },
                "ninja",
                "python",
                "patch",
                {"if": "unix", "then": ["make", "coreutils"]},
                "cmake",
                {"if": "build_platform != target_platform", "then": ["python"]},
                {
                    "if": "build_platform != target_platform",
                    "then": ["cross-python_${{ target_platform }}"],
                },
                {"if": "build_platform != target_platform", "then": ["cython"]},
                {"if": "build_platform != target_platform", "then": ["numpy"]},
                {"if": "build_platform != target_platform", "then": ["pybind11"]},
            ],
            "host": [],
            "run": [],
        },
    }

    if test := vinca_conf.get("_tests", {}).get(final_name):
        # parse as yaml
        text = test.read_text()
        test_content = ruamel.yaml.safe_load(text)
        recipe["test"] = test_content

    for p in pkg["authors"]:
        name = p.name + " (" + p.email + ")" if p.email else p.name
        recipe["about"]["maintainers"].append(name)

    for u in pkg["urls"]:
        if u.type == "website":
            recipe["about"]["homepage"] = u.url

    repository = distro.get_repository_url(pkg.name, pkg["urls"])
    if repository:
        recipe["about"]["repository"] = repository

    if not recipe["source"].get("git", None):
        aux = path.split("/")
        print(aux[: len(aux) - 1])
        recipe["source"]["path"] = "/".join(aux[: len(aux) - 1])
        recipe["source"]["target_directory"] = f"{final_name}/src/work"

    for d in pkg["buildtool_depends"]:
        recipe["requirements"]["host"].extend(
            resolve_pkgname(d.name, vinca_conf, distro)
        )

    for d in pkg["build_depends"]:
        recipe["requirements"]["host"].extend(
            resolve_pkgname(d.name, vinca_conf, distro)
        )

    for d in pkg["build_export_depends"]:
        recipe["requirements"]["host"].extend(
            resolve_pkgname(d.name, vinca_conf, distro)
        )
        recipe["requirements"]["run"].extend(
            resolve_pkgname(d.name, vinca_conf, distro)
        )

    for d in pkg["buildtool_export_depends"]:
        recipe["requirements"]["host"].extend(
            resolve_pkgname(d.name, vinca_conf, distro)
        )
        recipe["requirements"]["run"].extend(
            resolve_pkgname(d.name, vinca_conf, distro)
        )

    for d in pkg["test_depends"]:
        recipe["requirements"]["host"].extend(
            resolve_pkgname(d.name, vinca_conf, distro)
        )

    for d in pkg["exec_depends"]:
        recipe["requirements"]["run"].extend(
            resolve_pkgname(d.name, vinca_conf, distro)
        )

    if pkg.get_build_type() in ["cmake", "catkin"]:
        recipe["build"]["script"] = (
            "${{ '$RECIPE_DIR/build_catkin.sh' if unix or wasm32 else '%RECIPE_DIR%\\\\bld_catkin.bat' }}"
        )

    if pkg.get_build_type() in ["cmake", "catkin", "ament_cmake"]:
        recipe["requirements"]["build"].append(
            {
                "if": "osx",
                "then": [
                    "clang-tools ${{ (cxx_compiler_version ~ '.*') if "
                    "cxx_compiler_version is defined else '*' }}"
                ],
            }
        )

    return recipe


def print_generation_summary(distro, vinca_conf, outputs):
    """Print a summary of the generated recipes and, for each, *why* it was
    generated: whether it was requested directly by the config and which
    already-selected packages depend on it."""
    from rich.console import Console
    from rich.table import Table

    provenance = vinca_conf.get("_pkg_provenance", {})
    requested = provenance.get("requested_by_config", set())
    required_by = provenance.get("required_by", {})

    # names that actually produced a recipe in this run
    generated_outputs = {output["package"]["name"]: output for output in outputs}
    generated_names = set(generated_outputs)
    matched_names = set()

    rows = []
    for shortname in vinca_conf.get("_selected_pkgs", []):
        try:
            pkg_names = resolve_pkgname(shortname, vinca_conf, distro)
        except Exception:
            pkg_names = []
        if not pkg_names or pkg_names[0] not in generated_names:
            continue
        name = pkg_names[0]
        matched_names.add(name)

        is_requested = shortname in requested
        deps = sorted(required_by.get(shortname, set()))
        if deps:
            depended_on = ", ".join(deps[:8])
            if len(deps) > 8:
                depended_on += f", ... (+{len(deps) - 8} more)"
        else:
            depended_on = ""
        rows.append((name, is_requested, depended_on))

    # auxiliary recipes that don't map back to a selected package (e.g. mutex)
    leftovers = sorted(generated_names - matched_names)

    table = Table(
        title="Generated recipes and why they were selected",
        title_style="bold",
        header_style="bold",
    )
    table.add_column("Recipe", style="cyan", no_wrap=True)
    table.add_column("Requested by config", justify="center")
    table.add_column("Depended on by")

    for name, is_requested, depended_on in sorted(rows):
        table.add_row(
            name,
            "[green]yes[/green]" if is_requested else "[dim]no[/dim]",
            depended_on or "[dim]-[/dim]",
        )
    for name in leftovers:
        if is_legacy_compatibility_output(generated_outputs[name], distro, vinca_conf):
            reason = "[dim]legacy compatibility package[/dim]"
        else:
            reason = "[dim]auxiliary (e.g. mutex)[/dim]"
        table.add_row(name, "[dim]no[/dim]", reason)

    console = Console()
    console.print(table)
    console.print(f"Total generated recipes: [bold]{len(rows) + len(leftovers)}[/bold]")


def main():
    global distro, unsatisfied_deps

    arguments = parse_command_line(sys.argv)

    base_dir = os.path.abspath(arguments.dir)
    vinca_yaml = os.path.join(base_dir, "vinca.yaml")
    vinca_conf = read_vinca_yaml(vinca_yaml)

    if arguments.trigger_new_versions:
        vinca_conf["trigger_new_versions"] = True
    else:
        vinca_conf["trigger_new_versions"] = vinca_conf.get(
            "trigger_new_versions", False
        )

    if arguments.package:
        pkg_files = glob.glob(arguments.package)

        python_version = None
        if "python_version" in vinca_conf:
            python_version = vinca_conf["python_version"]

        distro = Distro(
            vinca_conf["ros_distro"],
            python_version,
            vinca_conf["_snapshot"],
            vinca_conf["_additional_packages_snapshot"],
        )
        additional_pkgs, parsed_pkgs = [], []
        for f in pkg_files:
            parsed_pkg = catkin_pkg.package.parse_package(f)
            additional_pkgs.append(parsed_pkg.name)
            parsed_pkgs.append(parsed_pkg)

        distro.add_packages(additional_pkgs)

        outputs = []
        for f in pkg_files:
            pkg = catkin_pkg.package.parse_package(f)
            recipe = parse_package(pkg, distro, vinca_conf, f)
            recipes = [recipe]
            compatibility_recipe = generate_legacy_compatibility_output(
                recipe, pkg.name, distro, vinca_conf
            )
            if compatibility_recipe is not None:
                recipes.append(compatibility_recipe)

            if arguments.multiple_file:
                for generated_recipe in recipes:
                    write_recipe_package(generated_recipe)
            else:
                outputs.extend(recipes)

        if not arguments.multiple_file:
            sources = {}
            for output in outputs:
                source = output.pop("source", None)
                if source is not None:
                    sources[output["package"]["name"]] = source
            write_recipe(sources, outputs, vinca_conf, distro)

    else:
        if arguments.skip_already_built_repodata or vinca_conf.get("skip_existing"):
            skip_built_packages = set()
            fn = arguments.skip_already_built_repodata
            if not fn:
                fn = vinca_conf.get("skip_existing")

            yaml = ruamel.yaml.YAML()
            additional_recipe_names = set()
            for add_rec in glob.glob(
                os.path.join(base_dir, "additional_recipes", "**", "recipe.yaml")
            ):
                with open(add_rec) as fi:
                    add_rec_y = yaml.load(fi)
                if arguments.platform == "emscripten-wasm32":
                    additional_recipe_names.add(add_rec_y["package"]["name"])
                else:
                    if add_rec_y["package"]["name"] not in [
                        "ros2-rmw-wasm-cpp",
                        "ros2-wasm-cpp",
                        "ros2-dynmsg",
                        "ros2-test-wasm",
                        "ros-humble-rmw-wasm-cpp",
                        "ros-humble-wasm-cpp",
                        "ros-humble-dynmsg",
                        "ros-humble-test-wasm",
                    ]:
                        additional_recipe_names.add(add_rec_y["package"]["name"])

            print("Found additional recipes: ", additional_recipe_names)

            fns = list(fn)
            for fn in fns:
                selected_bn = None

                print(f"Fetching repodata: {fn}")
                repodata = get_repodata(fn, get_conda_subdir())
                # currently we don't check the build numbers of local repodatas,
                # only URLs
                if "://" in fn:
                    selected_bn = vinca_conf.get("build_number", 0)

                all_pkgs = repodata.get("packages", {})
                all_pkgs.update(repodata.get("packages.conda", {}))
                for _, pkg in all_pkgs.items():
                    is_built = False
                    if selected_bn is not None:
                        pkg_build_number = get_pkg_build_number(
                            selected_bn, pkg["name"], vinca_conf
                        )
                        if pkg["build_number"] == pkg_build_number:
                            is_built = True
                    else:
                        is_built = True

                    if is_built:
                        print(f"Skipping {pkg['name']}")
                        if vinca_conf["trigger_new_versions"]:
                            skip_built_packages.add((pkg["name"], pkg["version"]))
                        else:
                            skip_built_packages.add(pkg["name"])

                vinca_conf["skip_built_packages"] = skip_built_packages
        else:
            vinca_conf["skip_built_packages"] = []
        print("Skip built packages!", vinca_conf["skip_built_packages"])
        python_version = None
        if "python_version" in vinca_conf:
            python_version = vinca_conf["python_version"]

        distro = Distro(
            vinca_conf["ros_distro"],
            python_version,
            vinca_conf["_snapshot"],
            vinca_conf["_additional_packages_snapshot"],
        )

        selected_pkgs = get_selected_packages(distro, vinca_conf)

        vinca_conf["_selected_pkgs"] = selected_pkgs

        if arguments.source:
            source = generate_source_version(distro, vinca_conf)
            outputs = generate_outputs_version(distro, vinca_conf)
        else:
            source = generate_source(distro, vinca_conf)
            outputs = generate_outputs(distro, vinca_conf)

        if arguments.multiple_file:
            write_recipe(source, outputs, vinca_conf, distro, False)
        else:
            write_recipe(source, outputs, vinca_conf, distro)

        print_generation_summary(distro, vinca_conf, outputs)

        if unsatisfied_deps:
            print("Unsatisfied dependencies:", unsatisfied_deps)

    print("build scripts are created successfully.")
