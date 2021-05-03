#!/usr/bin/env python

import argparse
import requests
import catkin_pkg
import sys
import os
import json
import glob
import platform
import ruamel.yaml
from pathlib import Path

from vinca import __version__
from .resolve import get_conda_index
from .resolve import resolve_pkgname
from .resolve import resolve_pkgname_from_indexes
from .template import write_recipe, write_recipe_package
from .distro import Distro

from vinca import config

unsatisfied_deps = set()
distro = None

parsed_args = None


def ensure_list(l):
    if not l:
        return []
    return l


def get_conda_subdir():
    if parsed_args.platform:
        return parsed_args.platform

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
      See: https://github.com/ros-forge/vinca
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
    global parsed_args, selected_platform
    parsed_args = arguments
    config.selected_platform = get_conda_subdir()
    return arguments


def get_depmods(vinca_conf, pkg_name):
    depmods = vinca_conf["depmods"].get(pkg_name, {})
    rm_deps, add_deps = {"build": set(), "run": set()}, {"build": set(), "run": set()}

    if depmods.get("remove"):
        for el in depmods["remove"]:
            if type(el) is str:
                rm_deps["build"].add(el)
                rm_deps["run"].add(el)
            elif type(el) == dict:
                rm_deps["build"] = set(el.get("build", []))
                rm_deps["run"] = set(el.get("run", []))

    if depmods.get("add"):
        for el in depmods["add"]:
            if type(el) is str:
                add_deps["build"].add(el)
                add_deps["run"].add(el)
            elif type(el) == dict:
                add_deps["build"] = set(el.get("build", []))
                add_deps["run"] = set(el.get("run", []))

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
        patches[splitted[0]]["any"].append(x)

    vinca_conf["_patches"] = patches

    if (patch_dir / "dependencies.yaml").exists():
        vinca_conf["depmods"] = yaml.load(open(patch_dir / "dependencies.yaml"))
    else:
        vinca_conf["depmods"] = {}

    config.ros_distro = vinca_conf["ros_distro"]
    config.skip_testing = vinca_conf.get("skip_testing", True)

    return vinca_conf


def generate_output(pkg_shortname, vinca_conf, distro, version):
    if pkg_shortname not in vinca_conf["_selected_pkgs"]:
        return None

    pkg_names = resolve_pkgname(pkg_shortname, vinca_conf, distro)
    if not pkg_names:
        return None

    if pkg_names[0] in vinca_conf["skip_built_packages"]:
        return None

    output = {
        "package": {
            "name": pkg_names[0],
            "version": version,
        },
        "requirements": {
            "build": [
                "{{ compiler('cxx') }}",
                "{{ compiler('c') }}",
                "ninja",
                {"sel(unix)": "make"},
                "cmake",
                {"sel(build_platform != target_platform)": "python"},
                {"sel(build_platform != target_platform)": "cross-python_{{ target_platform }}"},
                {"sel(build_platform != target_platform)": "cython"},
                {"sel(build_platform != target_platform)": "numpy"},
                {"sel(build_platform != target_platform)": "pybind11"},
            ],
            "host": [],
            "run": [],
        },
        "build": {"script": ""},
    }

    if pkg_shortname == "eigenpy" or pkg_shortname.replace("-", "_") == "slam_toolbox":
        output["requirements"]["build"] += ["pkg-config"]
    if pkg_shortname.replace("-", "_") == "ur_client_library":
        output["requirements"]["host"] += ["ros-noetic-catkin"]
    if pkg_shortname.replace("-", "_") == "mqtt_bridge":
        output["requirements"]["run"] += ["inject", "msgpack-python", "paho-mqtt", "pymongo"]
    if pkg_shortname.replace("-", "_") == "sainsmart_relay_usb" or pkg_shortname.replace("-", "_") == "kobuki_ftdi" or pkg_shortname.replace("-", "_") == "sick_tim" or pkg_shortname == "mrpt2":
        output["requirements"]["build"] += [{"sel(linux)": "{{ cdt('libudev') }}"}, {"sel(linux)": "{{ cdt('libudev-devel') }}"}]
    if pkg_shortname == "mrpt2":
        output["requirements"]["host"] += ["tinyxml2", "boost-cpp", "jsoncpp", "gtest", "boost", "libdc1394", "xorg-libxcomposite", "ros-noetic-octomap", "libftdi"]
        output["requirements"]["run"] += ["tinyxml2", "boost-cpp", "jsoncpp", "gtest", "boost", "libdc1394", "xorg-libxcomposite", "ros-noetic-octomap", "libftdi"]
        output["requirements"]["build"] += [{"sel(linux)": "{{ cdt('libxcomposite-devel') }}"}]
    if pkg_shortname.replace("-", "_") == "jsk_recognition_utils":
        output["requirements"]["host"] += ["glew"]
        output["requirements"]["run"] += ["glew"]


    pkg = catkin_pkg.package.parse_package_string(
        distro.get_release_package_xml(pkg_shortname)
    )

    pkg.evaluate_conditions(os.environ)

    resolved_python = resolve_pkgname("python", vinca_conf, distro)
    output["requirements"]["run"].extend(resolved_python)
    output["requirements"]["host"].extend(resolved_python)
    if pkg.get_build_type() in ["cmake", "catkin"]:
        # TODO find a way to get the conda "comments" with ruamel
        # output['script'] = ['bld_catkin.bat  # [win]', 'build_catkin.sh  # [unix]']
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
        resolved_setuptools = resolve_pkgname("setuptools", vinca_conf, distro)
        output["requirements"]["host"].extend(resolved_setuptools)
    else:
        return None

    if vinca_conf.get("mutex_package"):
        output["requirements"]["host"].append(vinca_conf["mutex_package"])
        output["requirements"]["run"].append(vinca_conf["mutex_package"])


    rm_deps, add_deps = get_depmods(vinca_conf, pkg.name)

    build_deps = pkg.build_depends
    build_deps += pkg.buildtool_depends
    build_deps += pkg.build_export_depends
    build_deps += pkg.buildtool_export_depends
    build_deps += pkg.test_depends
    build_deps = [d.name for d in build_deps if d.evaluated_condition]
    build_deps = (set(build_deps) - rm_deps["build"]) | add_deps["build"]

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
    run_deps = (set(run_deps) - rm_deps["run"]) | add_deps["run"]

    for dep in run_deps:
        if dep in ["REQUIRE_OPENGL", "REQUIRE_GL"]:
            output["requirements"]["host"].append(dep)
            continue

        resolved_dep = resolve_pkgname(dep, vinca_conf, distro, is_rundep=True)
        if not resolved_dep:
            unsatisfied_deps.add(dep)
            continue
        output["requirements"]["run"].extend(resolved_dep)

    output["requirements"]["run"] = list(set(output["requirements"]["run"]))
    output["requirements"]["host"] = list(set(output["requirements"]["host"]))
    output["requirements"]["run"] = sorted(output["requirements"]["run"])
    output["requirements"]["host"] = sorted(output["requirements"]["host"])

    output["requirements"]["run"] += [
        {
            "sel(osx and x86_64)": "__osx >={{ MACOSX_DEPLOYMENT_TARGET|default('10.14') }}"
        }
    ]

    # fix up OPENGL support for Unix
    if (
        "REQUIRE_OPENGL" in output["requirements"]["run"]
        or "REQUIRE_OPENGL" in output["requirements"]["host"]
    ):
        # add requirements for opengl
        if "REQUIRE_OPENGL" in output["requirements"]["run"]:
            output["requirements"]["run"].remove("REQUIRE_OPENGL")
        if "REQUIRE_OPENGL" in output["requirements"]["host"]:
            output["requirements"]["host"].remove("REQUIRE_OPENGL")

        output["requirements"]["build"] += [
            {"sel(unix)": "{{ cdt('mesa-libgl-devel') }}"},
            {"sel(unix)": "{{ cdt('mesa-dri-drivers') }}"},
            {"sel(linux)": "{{ cdt('libselinux') }}"},
            {"sel(linux)": "{{ cdt('libxdamage') }}"},
            {"sel(linux)": "{{ cdt('libxxf86vm') }}"},
            {"sel(linux)": "{{ cdt('libxfixes') }}"},
            {"sel(linux)": "{{ cdt('libxext') }}"},
            {"sel(linux)": "{{ cdt('libxau') }}"},
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
        if "REQUIRE_GL" in output["requirements"]["run"]:
            output["requirements"]["run"].remove("REQUIRE_GL")
        if "REQUIRE_GL" in output["requirements"]["host"]:
            output["requirements"]["host"].remove("REQUIRE_GL")

        output["requirements"]["build"] += [
            {"sel(unix)": "{{ cdt('mesa-libgl-devel') }}"},
            {"sel(unix)": "{{ cdt('mesa-dri-drivers') }}"},
            {"sel(linux)": "{{ cdt('libselinux') }}"},
            {"sel(linux)": "{{ cdt('libxxf86vm') }}"},
        ]

    return output


def generate_outputs(distro, vinca_conf):
    outputs = []
    for pkg_shortname in vinca_conf["_selected_pkgs"]:
        output = generate_output(
            pkg_shortname, vinca_conf, distro, distro.get_version(pkg_shortname)
        )
        if output is not None:
            outputs.append(output)
    return outputs


def generate_outputs_version(distro, vinca_conf):
    outputs = []
    for pkg_shortname in vinca_conf["_selected_pkgs"]:
        version = distro.get_version(pkg_shortname)
        if (
            vinca_conf["package_version"]
            and vinca_conf["package_version"][pkg_shortname]
        ):
            version = vinca_conf["package_version"][pkg_shortname]["version"]

        output = generate_output(pkg_shortname, vinca_conf, distro, version)
        if output is not None:
            outputs.append(output)

    return outputs


def generate_fat_output(pkg_shortname, vinca_conf, distro):
    if pkg_shortname not in vinca_conf["_selected_pkgs"]:
        return [], []
    pkg_names = resolve_pkgname(pkg_shortname, vinca_conf, distro)
    if not pkg_names:
        return [], []
    pkg = catkin_pkg.package.parse_package_string(
        distro.get_release_package_xml(pkg_shortname)
    )
    pkg.evaluate_conditions(os.environ)
    resolved_python = resolve_pkgname_from_indexes(
        "python", vinca_conf["_conda_indexes"]
    )
    host_requirements = []
    run_requirements = []
    run_requirements.extend(resolved_python)
    host_requirements.extend(resolved_python)
    if distro.get_python_version() == 3:
        resolved_setuptools = resolve_pkgname_from_indexes(
            "setuptools", vinca_conf["_conda_indexes"]
        )
        host_requirements.extend(resolved_setuptools)
    if not distro.check_ros1():
        resolved_colcon = resolve_pkgname_from_indexes(
            "colcon-common-extensions", vinca_conf["_conda_indexes"]
        )
        host_requirements.extend(resolved_colcon)

    build_deps = pkg.build_depends
    build_deps += pkg.buildtool_depends
    build_deps += pkg.build_export_depends
    build_deps += pkg.buildtool_export_depends
    build_deps += pkg.test_depends
    build_deps += pkg.run_depends
    build_deps += pkg.exec_depends
    build_deps = [d.name for d in build_deps if d.evaluated_condition]
    build_deps = set(build_deps)

    for dep in build_deps:
        if dep in vinca_conf["_selected_pkgs"]:
            # don't repeat the selected pkgs in the reqs.
            continue
        resolved_dep = resolve_pkgname_from_indexes(dep, vinca_conf["_conda_indexes"])
        if not resolved_dep:
            unsatisfied_deps.add(dep)
            continue
        host_requirements.extend(resolved_dep)

    run_deps = pkg.run_depends
    run_deps += pkg.exec_depends
    run_deps += pkg.build_export_depends
    run_deps += pkg.buildtool_export_depends
    run_deps = [d.name for d in run_deps if d.evaluated_condition]
    run_deps = set(run_deps)

    for dep in run_deps:
        if dep in vinca_conf["_selected_pkgs"]:
            # don't repeat the selected pkgs in the reqs.
            continue
        resolved_dep = resolve_pkgname_from_indexes(dep, vinca_conf["_conda_indexes"])
        if not resolved_dep:
            unsatisfied_deps.add(dep)
            continue
        run_requirements.extend(resolved_dep)

    return host_requirements, run_requirements


def generate_fat_outputs(distro, vinca_conf):
    outputs = []
    output = {
        "name": vinca_conf["name"],
        "requirements": {
            "build": ["{{ compiler('cxx') }}", "{{ compiler('c') }}", "ninja", "cmake"],
            "host": [],
            "run": [],
        },
    }

    # use catkin for ros1
    if distro.check_ros1():
        output["script"] = "bld_catkin_merge.bat"
    else:
        output["script"] = "bld_colcon_merge.bat"

    for pkg_shortname in vinca_conf["_selected_pkgs"]:
        host_requirements, run_requirements = generate_fat_output(
            pkg_shortname, vinca_conf, distro
        )
        output["requirements"]["host"].extend(host_requirements)
        output["requirements"]["run"].extend(run_requirements)

    output["requirements"]["host"] = list(set(output["requirements"]["host"]))
    output["requirements"]["run"] = list(set(output["requirements"]["run"]))
    output["requirements"]["host"] = sorted(output["requirements"]["host"])
    output["requirements"]["run"] = sorted(output["requirements"]["run"])
    outputs.append(output)

    return outputs


def generate_source(distro, vinca_conf):
    source = {}
    for pkg_shortname in vinca_conf["_selected_pkgs"]:
        url, version = distro.get_released_repo(pkg_shortname)
        entry = {}
        entry["git_url"] = url
        entry["git_rev"] = version
        pkg_names = resolve_pkgname(pkg_shortname, vinca_conf, distro)
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
                print(patches)
                common_prefix = os.path.commonprefix((os.getcwd(), patches[0]))
                print(common_prefix)
                entry["patches"] = [os.path.relpath(p, common_prefix) for p in patches]

        source[pkg_name] = entry

    return source


def generate_source_version(distro, vinca_conf):
    source = {}
    for pkg_shortname in vinca_conf["_selected_pkgs"]:
        url, version = distro.get_released_repo(pkg_shortname)
        if (
            vinca_conf["package_version"]
            and vinca_conf["package_version"][pkg_shortname]
        ):
            url = vinca_conf["package_version"][pkg_shortname]["url"]
            version = vinca_conf["package_version"][pkg_shortname]["version"]

        entry = {}
        entry["git_url"] = url
        entry["git_rev"] = version
        pkg_names = resolve_pkgname(pkg_shortname, vinca_conf, distro)
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
        for i in vinca_conf["packages_select_by_deps"]:
            i = i.replace("-", "_")
            selected_packages = selected_packages.union([i])
            if "skip_all_deps" not in vinca_conf or not vinca_conf["skip_all_deps"]:
                try:
                    pkgs = distro.get_depends(i)
                except KeyError:
                    # handle (rare) package names that use "-" as separator
                    pkgs = distro.get_depends(i.replace("_", "-"))
                    selected_packages.remove(i)
                    selected_packages.add(i.replace("_", "-"))
                selected_packages = selected_packages.union(pkgs)

    if (
        "packages_skip_by_deps" in vinca_conf
        and vinca_conf["packages_skip_by_deps"] is not None
    ):
        for i in vinca_conf["packages_skip_by_deps"]:
            i = i.replace("-", "_")
            skipped_packages = skipped_packages.union([i])
            try:
                pkgs = distro.get_depends(i)
            except KeyError:
                # handle (rare) package names that use "-" as separator
                pkgs = distro.get_depends(i.replace("_", "-"))
                selected_packages.remove(i)
                selected_packages.add(i.replace("_", "-"))
            skipped_packages = skipped_packages.union(pkgs)

    result = selected_packages.difference(skipped_packages)
    result = sorted(list(result))
    return result


# def parse_dep(dep, vinca_conf, distro):
#     res = dep.name
#     res = resolve_pkgname(res, vinca_conf, distro)

#     if dep.version_eq :
#         return res + " ==" + dep.version_eq

#     if dep.version_gt :
#         res = res + " >" + dep.version_gt

#         if dep.version_lt :
#             res = res + ", <" + dep.version_lt

#         if dep.version_lte :
#             res = res + ", <=" + dep.version_lte

#         return res

#     if dep.version_gte :
#         res = res + " >=" + dep.version_gte

#         if dep.version_lt :
#             res = res + ", <" + dep.version_lt

#         return res

#     if dep.version_lt :
#         res = res + " <" + dep.version_lt

#         if dep.version_gt :
#             res = res + ", >" + dep.version_gt

#         if dep.version_gte :
#             res = res + ", >=" + dep.version_gte

#         return res

#     if dep.version_lte :
#         res = res + " <=" + dep.version_lte

#         if dep.version_gt :
#             res = res + ", >" + dep.version_gt

#         return res

#     return res


def parse_package(pkg, distro, vinca_conf, path):

    name = pkg["name"].replace("_", "-")
    final_name = f"ros-{distro.name}-{name}"
    recipe = {
        "package": {"name": final_name, "version": pkg["version"]},
        "about": {
            "home": "https://www.ros.org/",
            "license": [str(l) for l in pkg["licenses"]],
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
                "ninja",
                {"sel(unix)": "make"},
                "cmake",
                {"sel(build_platform != target_platform)": "python"},
                {"sel(build_platform != target_platform)": "cross-python_{{ target_platform }}"},
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

    if name == "eigenpy":
        recipe["requirements"]["build"] += ["pkg-config"]

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
        if "REQUIRE_OPENGL" in recipe["requirements"]["run"]:
            recipe["requirements"]["run"].remove("REQUIRE_OPENGL")
        if "REQUIRE_OPENGL" in recipe["requirements"]["host"]:
            recipe["requirements"]["host"].remove("REQUIRE_OPENGL")

        recipe["requirements"]["build"] += [
            {"sel(unix)": "{{ cdt('mesa-libgl-devel') }}"},
            {"sel(unix)": "{{ cdt('mesa-dri-drivers') }}"},
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
        if "REQUIRE_GL" in recipe["requirements"]["run"]:
            recipe["requirements"]["run"].remove("REQUIRE_GL")
        if "REQUIRE_GL" in recipe["requirements"]["host"]:
            recipe["requirements"]["host"].remove("REQUIRE_GL")

        recipe["requirements"]["build"] += [
            {"sel(unix)": "{{ cdt('mesa-libgl-devel') }}"},
            {"sel(unix)": "{{ cdt('mesa-dri-drivers') }}"},
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
    vinca_conf["_conda_indexes"] = get_conda_index(vinca_conf, base_dir)

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

    if arguments.package:
        pkg_files = glob.glob(arguments.package)

        python_version = None
        if "python_version" in vinca_conf:
            python_version = vinca_conf["python_version"]

        distro = Distro(vinca_conf["ros_distro"], python_version)
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

            fns = list(fn)
            for fn in fns:
                selected_bn = None
                if "://" in fn:
                    fn += f"{get_conda_subdir()}/repodata.json"
                    request = requests.get(fn)
                    print(f"Fetching repodata: {fn}")

                    repodata = request.json()
                    selected_bn = vinca_conf.get("build_number", 0)
                    distro = vinca_conf["ros_distro"]
                    for pkg_name, pkg in repodata.get("packages").items():
                        if pkg_name.startswith(f"ros-{distro}"):
                            print(f"Already built {pkg_name}")
                            selected_bn = max(selected_bn, pkg["build_number"])
                else:
                    with open(fn) as fi:
                        repodata = json.load(fi)

                print(f"Selected build number: {selected_bn}")

                explicitly_selected_pkgs = [f"ros-{distro}-{pkg.replace('_', '-')}" for pkg in ensure_list(vinca_conf["packages_select_by_deps"])]

                for _, pkg in repodata.get("packages").items():
                    if selected_bn is not None:
                        if vinca_conf.get("full_rebuild", True):
                            if pkg["build_number"] == selected_bn:
                                skip_built_packages.add(pkg["name"])
                        else:
                            # remove all packages except explicitly selected ones
                            if pkg["name"] not in explicitly_selected_pkgs or pkg["build_number"] == selected_bn:
                                skip_built_packages.add(pkg["name"])
                    else:
                        skip_built_packages.add(pkg["name"])

                vinca_conf["skip_built_packages"] = skip_built_packages
        else:
            vinca_conf["skip_built_packages"] = []

        python_version = None
        if "python_version" in vinca_conf:
            python_version = vinca_conf["python_version"]

        distro = Distro(vinca_conf["ros_distro"], python_version)

        selected_pkgs = get_selected_packages(distro, vinca_conf)

        vinca_conf["_selected_pkgs"] = selected_pkgs

        if "fat_archive" in vinca_conf and vinca_conf["fat_archive"]:
            source = generate_fat_source(distro, vinca_conf)
            outputs = generate_fat_outputs(distro, vinca_conf)
        else:
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

        print(unsatisfied_deps)

    print("build scripts are created successfully.")
