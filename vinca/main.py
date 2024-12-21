#!/usr/bin/env python

import argparse
import catkin_pkg
import sys
import os
import glob
import platform
import ruamel.yaml
from pathlib import Path

from vinca import __version__
from .resolve import get_conda_index
from .resolve import resolve_pkgname
from .template import write_recipe, write_recipe_package
from .distro import Distro

from vinca import config
from vinca.utils import get_repodata

unsatisfied_deps = set()
distro = None


def ensure_list(obj):
    if not obj:
        return []
    assert isinstance(obj, list)
    return obj


def get_conda_subdir():
    if config.parsed_args.platform:
        return config.parsed_args.platform

    sys_platform = sys.platform
    machine = platform.machine()
    if sys_platform.startswith("linux"):
        if machine == "aarch64":
            return "linux-aarch64"
        elif machine == "x86_64":
            return "linux-64"
        else:
            raise RuntimeError("Unknown machine!")
    elif sys_platform == "darwin":
        if machine == "arm64":
            return "osx-arm64"
        else:
            return "osx-64"
    elif sys_platform == "win32":
        return "win-64"


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
    parser.add_argument(
        "-z",
        "--snapshot",
        dest="snapshot",
        default=None,
        help="The version snapshot file (default: None)."
    )
    arguments = parser.parse_args(argv[1:])
    global selected_platform
    config.parsed_args = arguments
    config.selected_platform = get_conda_subdir()
    return arguments


def get_depmods(vinca_conf, pkg_name):
    depmods = vinca_conf["depmods"].get(pkg_name, {})
    rm_deps, add_deps = (
        {"build": [], "host": [], "run": []},
        {"build": [], "host": [], "run": []},
    )

    for dep_type in ["build", "host", "run"]:
        if depmods.get("remove_" + dep_type):
            for el in depmods["remove_" + dep_type]:
                if isinstance(el, dict):
                    rm_deps[dep_type].append(dict(el))
                else:
                    rm_deps[dep_type].append(el)

        if depmods.get("add_" + dep_type):
            for el in depmods["add_" + dep_type]:
                if isinstance(el, dict):
                    add_deps[dep_type].append(dict(el))
                else:
                    add_deps[dep_type].append(el)
    return rm_deps, add_deps


def read_vinca_yaml(filepath):
    yaml = ruamel.yaml.YAML()
    vinca_conf = yaml.load(open(filepath, "r"))

    # normalize paths to absolute paths
    conda_index = []
    for i in vinca_conf["conda_index"]:
        if os.path.isfile(i):
            conda_index.append(os.path.abspath(i))
        else:
            conda_index.append(i)

    vinca_conf["conda_index"] = conda_index
    patch_dir = Path(vinca_conf["patch_dir"]).absolute()
    vinca_conf["_patch_dir"] = patch_dir
    patches = {}

    for x in glob.glob(os.path.join(vinca_conf["_patch_dir"], "*.patch")):
        splitted = os.path.basename(x).split(".")
        if splitted[0] not in patches:
            patches[splitted[0]] = {"any": [], "osx": [], "linux": [], "win": []}
        if len(splitted) == 3:
            if splitted[1] in ("osx", "linux", "win"):
                patches[splitted[0]][splitted[1]].append(x)
                continue
            if splitted[1] == "unix":
                patches[splitted[0]]["linux"].append(x)
                patches[splitted[0]]["osx"].append(x)
                continue

        patches[splitted[0]]["any"].append(x)

    vinca_conf["_patches"] = patches

    if (patch_dir / "dependencies.yaml").exists():
        vinca_conf["depmods"] = yaml.load(open(patch_dir / "dependencies.yaml"))
    if not vinca_conf.get("depmods"):
        vinca_conf["depmods"] = {}

    config.ros_distro = vinca_conf["ros_distro"]
    config.skip_testing = vinca_conf.get("skip_testing", True)

    vinca_conf["_conda_indexes"] = get_conda_index(
        vinca_conf, os.path.dirname(filepath)
    )

    vinca_conf["trigger_new_versions"] = vinca_conf.get("trigger_new_versions", False)

    return vinca_conf


def read_snapshot(filepath):
    if not filepath:
        return None

    yaml = ruamel.yaml.YAML()
    snapshot = yaml.load(open(filepath, "r"))
    return snapshot


def generate_output(pkg_shortname, vinca_conf, distro, version, all_pkgs=None):
    if not all_pkgs:
        all_pkgs = []

    if pkg_shortname not in vinca_conf["_selected_pkgs"]:
        return None

    pkg_names = resolve_pkgname(pkg_shortname, vinca_conf, distro)
    if not pkg_names:
        return None

    if vinca_conf.get("trigger_new_versions"):
        if (pkg_names[0], version) in vinca_conf["skip_built_packages"]:
            return None
    else:
        if pkg_names[0] in vinca_conf["skip_built_packages"]:
            return None

    # TODO: Remove hardcoded cmake version after building new versions of ament_cmake_export_target
    # see: https://github.com/ament/ament_cmake/commit/796cef7d7df2ddb806f774a9889e608cc82285d3
    output = {
        "package": {"name": pkg_names[0], "version": version},
        "requirements": {
            "build": [
                "{{ compiler('cxx') }}",
                "{{ compiler('c') }}",
                {"sel(linux64)": "sysroot_linux-64 2.17"},
                "ninja",
                "setuptools",
                {"sel(unix)": "make"},
                {"sel(unix)": "coreutils"},
                {"sel(osx)": "tapi"},
                {"sel(build_platform != target_platform)": "pkg-config"},
                "cmake",
                "cython",
                {"sel(build_platform != target_platform)": "python"},
                {
                    "sel(build_platform != target_platform)": "cross-python_{{ target_platform }}"
                },
                {"sel(build_platform != target_platform)": "numpy"},
            ],
            "host": [
                {"sel(build_platform == target_platform)": "pkg-config"},
                "numpy",
                "pip",
            ],
            "run": [],
        },
        "build": {"script": ""},
    }

    pkg = catkin_pkg.package.parse_package_string(
        distro.get_release_package_xml(pkg_shortname)
    )

    pkg.evaluate_conditions(os.environ)

    resolved_python = resolve_pkgname("python", vinca_conf, distro)
    output["requirements"]["run"].extend(resolved_python)
    output["requirements"]["host"].extend(resolved_python)
    if pkg.get_build_type() in ["cmake", "catkin"]:
        output["build"]["script"] = {
            "sel(win)": "bld_catkin.bat",
            "sel(unix)": "build_catkin.sh",
        }
    elif pkg.get_build_type() in ["ament_cmake"]:
        output["build"]["script"] = {
            "sel(win)": "bld_ament_cmake.bat",
            "sel(unix)": "build_ament_cmake.sh",
        }
    elif pkg.get_build_type() in ["ament_python"]:
        output["build"]["script"] = {
            "sel(win)": "bld_ament_python.bat",
            "sel(unix)": "build_ament_python.sh",
        }
        resolved_setuptools = resolve_pkgname("python-setuptools", vinca_conf, distro)
        output["requirements"]["host"].extend(resolved_setuptools)
    else:
        print(f"Unknown build type for {pkg_shortname}: {pkg.get_build_type()}")
        return None

    if vinca_conf.get("mutex_package"):
        output["requirements"]["host"].append(vinca_conf["mutex_package"])
        output["requirements"]["run"].append(vinca_conf["mutex_package"])

    if not distro.check_ros1() and pkg_shortname not in [
        "ament_cmake_core",
        "ament_package",
        "ros_workspace",
        "ros_environment",
    ]:
        output["requirements"]["host"].append(
            f"ros-{config.ros_distro}-ros-environment"
        )
        output["requirements"]["host"].append(f"ros-{config.ros_distro}-ros-workspace")
        output["requirements"]["run"].append(f"ros-{config.ros_distro}-ros-workspace")

    rm_deps, add_deps = get_depmods(vinca_conf, pkg.name)
    gdeps = []
    if pkg.group_depends:
        for gdep in pkg.group_depends:
            gdep.extract_group_members(all_pkgs)
            gdeps += gdep.members

    build_tool_deps = pkg.buildtool_depends
    build_tool_deps += pkg.buildtool_export_depends
    build_tool_deps = [d.name for d in build_tool_deps if d.evaluated_condition]

    build_deps = pkg.build_depends
    build_deps += pkg.build_export_depends
    build_deps += pkg.test_depends
    build_deps = [d.name for d in build_deps if d.evaluated_condition]
    build_deps += gdeps

    # we stick some build tools into the `build` section to make cross compilation work
    # right now it's only `git`.
    for dep in build_tool_deps:
        resolved_dep = resolve_pkgname(dep, vinca_conf, distro)
        if not resolved_dep:
            unsatisfied_deps.add(dep)
            continue

        if "git" in resolved_dep:
            output["requirements"]["build"].extend(resolved_dep)
        else:
            # remove duplicate cmake
            if dep not in ["cmake"]:
                build_deps.append(dep)

        # Hack to add cyclonedds into build for cross compilation
        if pkg_shortname == "cyclonedds" or "cyclonedds" in (
            build_deps + build_tool_deps
        ):
            output["requirements"]["build"].append(
                {
                    "sel(build_platform != target_platform)": f"ros-{config.ros_distro}-cyclonedds"
                }
            )

    for dep in build_deps:
        if dep in ["REQUIRE_OPENGL", "REQUIRE_GL"]:
            output["requirements"]["host"].append(dep)
            continue

        resolved_dep = resolve_pkgname(dep, vinca_conf, distro)
        if not resolved_dep:
            unsatisfied_deps.add(dep)
            continue
        output["requirements"]["host"].extend(resolved_dep)

    run_deps = pkg.run_depends
    run_deps += pkg.exec_depends
    run_deps += pkg.build_export_depends
    run_deps += pkg.buildtool_export_depends
    run_deps = [d.name for d in run_deps if d.evaluated_condition]
    run_deps += gdeps

    for dep in run_deps:
        if dep in ["REQUIRE_OPENGL", "REQUIRE_GL"]:
            output["requirements"]["host"].append(dep)
            continue

        resolved_dep = resolve_pkgname(dep, vinca_conf, distro, is_rundep=True)
        if not resolved_dep:
            unsatisfied_deps.add(dep)
            continue
        output["requirements"]["run"].extend(resolved_dep)

    for dep_type in ["build", "host", "run"]:
        for dep in add_deps[dep_type]:
            output["requirements"][dep_type].append(dep)
        for dep in rm_deps[dep_type]:
            while dep in output["requirements"][dep_type]:
                output["requirements"][dep_type].remove(dep)

    def sortkey(k):
        if isinstance(k, dict):
            return list(k.values())[0]
        return k

    output["requirements"]["run"] = sorted(output["requirements"]["run"], key=sortkey)
    output["requirements"]["host"] = sorted(output["requirements"]["host"], key=sortkey)

    output["requirements"]["run"] += [
        {
            "sel(osx and x86_64)": "__osx >={{ MACOSX_DEPLOYMENT_TARGET|default('10.14') }}"
        }
    ]

    if f"ros-{config.ros_distro}-pybind11-vendor" in output["requirements"]["host"]:
        output["requirements"]["host"] += ["pybind11"]
    if "pybind11" in output["requirements"]["host"]:
        output["requirements"]["build"] += [
            {"sel(build_platform != target_platform)": "pybind11"}
        ]
    # pkg-config + pyqt-builder + git + doxygen must be in build, not host for cross-compile
    if "doxygen" in output["requirements"]["host"]:
        output["requirements"]["build"] += [
            {"sel(build_platform != target_platform)": "doxygen"}
        ]
        while "doxygen" in output["requirements"]["host"]:
            output["requirements"]["host"].remove("doxygen")
        output["requirements"]["host"] += [
            {"sel(build_platform == target_platform)": "doxygen"}
        ]
    if "pyqt-builder" in output["requirements"]["host"]:
        output["requirements"]["build"] += [
            {"sel(build_platform != target_platform)": "pyqt-builder"}
        ]
        while "pyqt-builder" in output["requirements"]["host"]:
            output["requirements"]["host"].remove("pyqt-builder")
        output["requirements"]["host"] += [
            {"sel(build_platform == target_platform)": "pyqt-builder"}
        ]
    if "git" in output["requirements"]["host"]:
        output["requirements"]["build"] += [
            {"sel(build_platform != target_platform)": "git"}
        ]
        while "git" in output["requirements"]["host"]:
            output["requirements"]["host"].remove("git")
        output["requirements"]["host"] += [
            {"sel(build_platform == target_platform)": "git"}
        ]

    # fix up OPENGL support for Unix
    if (
        "REQUIRE_OPENGL" in output["requirements"]["run"]
        or "REQUIRE_OPENGL" in output["requirements"]["host"]
    ):
        # add requirements for opengl
        while "REQUIRE_OPENGL" in output["requirements"]["run"]:
            output["requirements"]["run"].remove("REQUIRE_OPENGL")
        while "REQUIRE_OPENGL" in output["requirements"]["host"]:
            output["requirements"]["host"].remove("REQUIRE_OPENGL")

        output["requirements"]["build"] += [
            {"sel(linux)": "{{ cdt('mesa-libgl-devel') }}"},
            {"sel(linux)": "{{ cdt('mesa-dri-drivers') }}"},
            {"sel(linux)": "{{ cdt('libselinux') }}"},
            {"sel(linux)": "{{ cdt('libxdamage') }}"},
            {"sel(linux)": "{{ cdt('libxxf86vm') }}"},
            {"sel(linux)": "{{ cdt('libxfixes') }}"},
            {"sel(linux)": "{{ cdt('libxext') }}"},
            {"sel(linux)": "{{ cdt('libxau') }}"},
            {"sel(linux)": "{{ cdt('libxcb') }}"},
        ]
        output["requirements"]["host"] += [
            {"sel(unix)": "xorg-libx11"},
            {"sel(unix)": "xorg-libxext"},
            # 'xorg-libxfixes  [unix]',
        ]
        output["requirements"]["run"] += [
            {"sel(unix)": "xorg-libx11"},
            {"sel(unix)": "xorg-libxext"},
            # 'xorg-libxfixes  [unix]',
        ]

    # fix up GL support for Unix
    if (
        "REQUIRE_GL" in output["requirements"]["run"]
        or "REQUIRE_GL" in output["requirements"]["host"]
    ):
        # add requirements for gl
        while "REQUIRE_GL" in output["requirements"]["run"]:
            output["requirements"]["run"].remove("REQUIRE_GL")
        while "REQUIRE_GL" in output["requirements"]["host"]:
            output["requirements"]["host"].remove("REQUIRE_GL")

        output["requirements"]["build"] += [
            {"sel(linux)": "{{ cdt('mesa-libgl-devel') }}"},
            {"sel(linux)": "{{ cdt('mesa-dri-drivers') }}"},
            {"sel(linux)": "{{ cdt('libselinux') }}"},
            {"sel(linux)": "{{ cdt('libxxf86vm') }}"},
        ]

    # remove duplicates
    for dep_type in ["build", "host", "run"]:
        tmp_nonduplicate = []
        [
            tmp_nonduplicate.append(x)
            for x in output["requirements"][dep_type]
            if x not in tmp_nonduplicate
        ]
        output["requirements"][dep_type] = tmp_nonduplicate

    return output


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
            outputs.append(output)
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
            outputs.append(output)

    return outputs


def generate_source(distro, vinca_conf):
    source = {}
    for pkg_shortname in vinca_conf["_selected_pkgs"]:
        if not distro.check_package(pkg_shortname):
            print(f"Could not generate source for {pkg_shortname}")
            continue

        url, version = distro.get_released_repo(pkg_shortname)
        entry = {}
        entry["git_url"] = url
        entry["git_rev"] = version
        pkg_names = resolve_pkgname(pkg_shortname, vinca_conf, distro)
        pkg_version = distro.get_version(pkg_shortname)
        print("Checking ", pkg_shortname, pkg_version)
        if not pkg_names:
            continue
        if vinca_conf.get("trigger_new_versions"):
            if (pkg_names[0], pkg_version) in vinca_conf["skip_built_packages"]:
                continue
        else:
            if pkg_names[0] in vinca_conf["skip_built_packages"]:
                continue
        pkg_name = pkg_names[0]
        entry["folder"] = "%s/src/work" % pkg_name

        patches = []
        pd = vinca_conf["_patches"].get(pkg_name)
        if pd:
            patches.extend(pd["any"])

            # find specific patches
            plat = get_conda_subdir().split("-")[0]
            patches.extend(pd[plat])
            if len(patches):
                print(patches)
                common_prefix = os.path.commonprefix((os.getcwd(), patches[0]))
                print(common_prefix)
                entry["patches"] = [os.path.relpath(p, common_prefix) for p in patches]

        source[pkg_name] = entry

    return source


def generate_source_version(distro, vinca_conf):
    source = {}
    for pkg_shortname in vinca_conf["_selected_pkgs"]:
        if not distro.check_package(pkg_shortname):
            print(f"Could not generate source for {pkg_shortname}")
            continue

        url, version = distro.get_released_repo(pkg_shortname)

        entry = {}
        entry["git_url"] = url
        entry["git_rev"] = version
        pkg_names = resolve_pkgname(pkg_shortname, vinca_conf, distro)
        if vinca_conf.get("trigger_new_versions"):
            if (
                not pkg_names
                or (pkg_names[0], version) in vinca_conf["skip_built_packages"]
            ):
                continue
        else:
            if not pkg_names or pkg_names[0] in vinca_conf["skip_built_packages"]:
                continue            
        pkg_name = pkg_names[0]
        entry["folder"] = "%s/src/work" % pkg_name

        patches = []
        pd = vinca_conf["_patches"].get(pkg_name)
        if pd:
            patches.extend(pd["any"])

            # find specific patches
            plat = get_conda_subdir().split("-")[0]
            patches.extend(pd[plat])
            if len(patches):
                entry["patches"] = patches

        source[pkg_name] = entry

    return source


def generate_fat_source(distro, vinca_conf):
    source = []
    for pkg_shortname in vinca_conf["_selected_pkgs"]:
        if not distro.check_package(pkg_shortname):
            print(f"Could not generate source for {pkg_shortname}")
            continue

        url, version = distro.get_released_repo(pkg_shortname)
        entry = {}
        entry["git_url"] = url
        entry["git_rev"] = version
        pkg_names = resolve_pkgname(pkg_shortname, vinca_conf, distro)
        if not pkg_names:
            continue
        pkg_name = pkg_names[0]
        entry["folder"] = "src/%s" % pkg_name
        patch_path = os.path.join(vinca_conf["_patch_dir"], "%s.patch" % pkg_name)
        if os.path.exists(patch_path):
            entry["patches"] = [
                "%s/%s" % (vinca_conf["patch_dir"], "%s.patch" % pkg_name)
            ]
        source.append(entry)
    return source


def get_selected_packages(distro, vinca_conf):
    selected_packages = set()
    skipped_packages = set()

    if vinca_conf.get("build_all", False):
        selected_packages = set(distro._distro.release_packages.keys())
    elif vinca_conf["packages_select_by_deps"]:

        if (
            "packages_skip_by_deps" in vinca_conf
            and vinca_conf["packages_skip_by_deps"] is not None
        ):
            for i in vinca_conf["packages_skip_by_deps"]:
                skipped_packages = skipped_packages.union([i, i.replace("-", "_")])
        print("Skipped pkgs: ", skipped_packages)
        for i in vinca_conf["packages_select_by_deps"]:
            i = i.replace("-", "_")
            selected_packages = selected_packages.union([i])
            if "skip_all_deps" not in vinca_conf or not vinca_conf["skip_all_deps"]:
                if i in skipped_packages:
                    continue
                try:
                    pkgs = distro.get_depends(i, ignore_pkgs=skipped_packages)
                except KeyError:
                    # handle (rare) package names that use "-" as separator
                    pkgs = distro.get_depends(i.replace("_", "-"))
                    selected_packages.remove(i)
                    selected_packages.add(i.replace("_", "-"))
                selected_packages = selected_packages.union(pkgs)

    result = sorted(list(selected_packages))
    return result


def parse_package(pkg, distro, vinca_conf, path):

    name = pkg["name"].replace("_", "-")
    final_name = f"ros-{distro.name}-{name}"
    recipe = {
        "package": {"name": final_name, "version": pkg["version"]},
        "about": {
            "home": "https://www.ros.org/",
            "license": [str(lic) for lic in pkg["licenses"]],
            "summary": pkg["description"],
            "maintainers": [],
        },
        "extra": {"recipe-maintainers": ["robostack"]},
        "build": {
            "number": 0,
            "script": {"sel(unix)": "build_catkin.sh", "sel(win)": "build_catkin.bat"},
        },
        "source": {},
        "requirements": {
            "build": [
                "{{ compiler('cxx') }}",
                "{{ compiler('c') }}",
                {"sel(linux64)": "sysroot_linux-64 2.17"},
                "ninja",
                {"sel(unix)": "make"},
                {"sel(unix)": "coreutils"},
                "cmake",
                {"sel(build_platform != target_platform)": "python"},
                {
                    "sel(build_platform != target_platform)": "cross-python_{{ target_platform }}"
                },
                {"sel(build_platform != target_platform)": "cython"},
                {"sel(build_platform != target_platform)": "numpy"},
                {"sel(build_platform != target_platform)": "pybind11"},
            ],
            "host": [],
            "run": [],
        },
    }

    for p in pkg["authors"]:
        name = p.name + " (" + p.email + ")" if p.email else p.name
        recipe["about"]["maintainers"].append(name)

    # for p in pkg['maintainers'] :
    #     name = p.name + " (" + p.email + ")" if p.email else p.name
    #     recipe['about']['maintainers'].append(name)

    for u in pkg["urls"]:
        # if u.type == 'repository' :
        #     recipe['source']['git_url'] = u.url
        #     recipe['source']['git_rev'] = recipe['package']['version']
        if u.type == "website":
            recipe["about"]["home"] = u.url

        # if u.type == 'bugtracker' :
        #    recipe['about']['url_issues'] = u.url

    if not recipe["source"].get("git_url", None):
        aux = path.split("/")
        print(aux[: len(aux) - 1])
        recipe["source"]["path"] = "/".join(aux[: len(aux) - 1])
        recipe["source"]["folder"] = f"{final_name}/src/work"

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
        recipe["build"]["script"] = {
            "sel(win)": "bld_catkin.bat",
            "sel(unix)": "build_catkin.sh",
        }

    # fix up OPENGL support for Unix
    if (
        "REQUIRE_OPENGL" in recipe["requirements"]["run"]
        or "REQUIRE_OPENGL" in recipe["requirements"]["host"]
    ):
        # add requirements for opengl
        while "REQUIRE_OPENGL" in recipe["requirements"]["run"]:
            recipe["requirements"]["run"].remove("REQUIRE_OPENGL")
        while "REQUIRE_OPENGL" in recipe["requirements"]["host"]:
            recipe["requirements"]["host"].remove("REQUIRE_OPENGL")

        recipe["requirements"]["build"] += [
            {"sel(linux)": "{{ cdt('mesa-libgl-devel') }}"},
            {"sel(linux)": "{{ cdt('mesa-dri-drivers') }}"},
            {"sel(linux)": "{{ cdt('libselinux') }}"},
            {"sel(linux)": "{{ cdt('libxdamage') }}"},
            {"sel(linux)": "{{ cdt('libxxf86vm') }}"},
            {"sel(linux)": "{{ cdt('libxfixes') }}"},
            {"sel(linux)": "{{ cdt('libxext') }}"},
            {"sel(linux)": "{{ cdt('libxau') }}"},
        ]
        recipe["requirements"]["host"] += [
            {"sel(unix)": "xorg-libx11"},
            {"sel(unix)": "xorg-libxext"},
        ]
        recipe["requirements"]["run"] += [
            {"sel(unix)": "xorg-libx11"},
            {"sel(unix)": "xorg-libxext"},
        ]

    # fix up GL support for Unix
    if (
        "REQUIRE_GL" in recipe["requirements"]["run"]
        or "REQUIRE_GL" in recipe["requirements"]["host"]
    ):
        # add requirements for gl
        while "REQUIRE_GL" in recipe["requirements"]["run"]:
            recipe["requirements"]["run"].remove("REQUIRE_GL")
        while "REQUIRE_GL" in recipe["requirements"]["host"]:
            recipe["requirements"]["host"].remove("REQUIRE_GL")

        recipe["requirements"]["build"] += [
            {"sel(linux)": "{{ cdt('mesa-libgl-devel') }}"},
            {"sel(linux)": "{{ cdt('mesa-dri-drivers') }}"},
            {"sel(linux)": "{{ cdt('libselinux') }}"},
            {"sel(linux)": "{{ cdt('libxxf86vm') }}"},
        ]

    return recipe


def main():
    global distro, unsatisfied_deps

    arguments = parse_command_line(sys.argv)

    base_dir = os.path.abspath(arguments.dir)
    vinca_yaml = os.path.join(base_dir, "vinca.yaml")
    vinca_conf = read_vinca_yaml(vinca_yaml)
    snapshot = read_snapshot(arguments.snapshot)

    from .template import generate_bld_ament_cmake
    from .template import generate_bld_ament_python
    from .template import generate_bld_catkin
    from .template import generate_activate_hook
    from .template import generate_bld_colcon_merge
    from .template import generate_bld_catkin_merge

    generate_bld_ament_cmake()
    generate_bld_ament_python()
    generate_bld_catkin()
    generate_bld_colcon_merge()
    generate_bld_catkin_merge()
    generate_activate_hook()

    if arguments.trigger_new_versions:
        vinca_conf["trigger_new_versions"] = True
    else:
        vinca_conf["trigger_new_versions"] = vinca_conf.get("trigger_new_versions", False)

    if arguments.package:
        pkg_files = glob.glob(arguments.package)

        python_version = None
        if "python_version" in vinca_conf:
            python_version = vinca_conf["python_version"]

        distro = Distro(vinca_conf["ros_distro"], python_version, snapshot)
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

            if arguments.multiple_file:
                write_recipe_package(recipe)
            else:
                outputs.append(recipe)

        if not arguments.multiple_file:
            sources = {}
            for o in outputs:
                sources[o["package"]["name"]] = o["source"]
                del o["source"]
            write_recipe(sources, outputs)

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
                    distro = vinca_conf["ros_distro"]
                    for pkg_name, pkg in repodata.get("packages").items():
                        if pkg_name.startswith(f"ros-{distro}"):
                            if pkg_name.rsplit("-", 2)[0] in additional_recipe_names:
                                print(
                                    f"Skipping additional recipe for build number computation {pkg_name}"
                                )
                                continue
                            selected_bn = max(selected_bn, pkg["build_number"])

                print(f"Selected build number: {selected_bn}")

                explicitly_selected_pkgs = [
                    f"ros-{distro}-{pkg.replace('_', '-')}"
                    for pkg in ensure_list(vinca_conf["packages_select_by_deps"])
                ]

                for _, pkg in repodata.get("packages").items():
                    is_built = False
                    if selected_bn is not None:
                        if vinca_conf.get("full_rebuild", True):
                            if pkg["build_number"] == selected_bn:
                                is_built = True
                        else:
                            # remove all packages except explicitly selected ones
                            if (
                                pkg["name"] not in explicitly_selected_pkgs
                                or pkg["build_number"] == selected_bn
                            ):
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

        distro = Distro(vinca_conf["ros_distro"], python_version, snapshot)

        selected_pkgs = get_selected_packages(distro, vinca_conf)

        vinca_conf["_selected_pkgs"] = selected_pkgs

        if arguments.source:
            source = generate_source_version(distro, vinca_conf)
            outputs = generate_outputs_version(distro, vinca_conf)
        else:
            source = generate_source(distro, vinca_conf)
            outputs = generate_outputs(distro, vinca_conf)

        if arguments.multiple_file:
            write_recipe(source, outputs, vinca_conf.get("build_number", 0), False)
        else:
            write_recipe(source, outputs, vinca_conf.get("build_number", 0))

        if unsatisfied_deps:
            print("Unsatisfied dependencies:", unsatisfied_deps)

    print("build scripts are created successfully.")
