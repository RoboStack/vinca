#!/usr/bin/env python

import argparse
import catkin_pkg
import sys
import os
from vinca import __version__
from .resolve import get_conda_index
from .resolve import resolve_pkgname
from .template import write_recipe
from .distro import Distro

unsatisfied_deps = set()
distro = None


def parse_command_line(argv):
    """
    Parse command line argument. See -h option.
    :param argv: the actual program arguments
    :return: parsed arguments
    """
    import textwrap

    default_dir = "."

    example = textwrap.dedent("""
      Examples:
        {0} -d ./examples/
      See: https://github.com/seanyen/vinca
    """).format(os.path.basename(argv[0]))
    formatter_class = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(
        description="Conda recipe generator for ROS packages",
        epilog=example,
        formatter_class=formatter_class)
    parser.add_argument(
        "-V", "--version", action="version",
        version="%(prog)s {}".format(__version__))
    parser.add_argument(
        "-d", "--dir", dest="dir", default=default_dir,
        help="The directory to process (default: {}).".format(default_dir))
    arguments = parser.parse_args(argv[1:])
    return arguments


def read_vinca_yaml(filepath):
    import ruamel.yaml

    yaml = ruamel.yaml.YAML()
    vinca_conf = yaml.load(open(filepath, 'r'))

    # normalize paths to absolute paths
    conda_index = []
    for i in vinca_conf['conda_index']:
        conda_index.append(os.path.abspath(i))
    vinca_conf['conda_index'] = conda_index
    vinca_conf['_patch_dir'] = os.path.abspath(vinca_conf['patch_dir'])
    return vinca_conf


def generate_output(pkg_shortname, vinca_conf, distro):
    if pkg_shortname not in vinca_conf['_selected_pkgs']:
        return None
    pkg_names = resolve_pkgname(pkg_shortname, vinca_conf, distro)
    if not pkg_names:
        return None
    output = {
        'name': pkg_names[0],
        'version': distro.get_version(pkg_shortname),
        'requirements': {
            'build': [
                "{{ compiler('cxx') }}",
                "{{ compiler('c') }}",
                "ninja",
                "cmake"
            ],
            'host': [],
            'run': []
        }
    }
    pkg = catkin_pkg.package.parse_package_string(
        distro.get_release_package_xml(pkg_shortname))
    pkg.evaluate_conditions(os.environ)
    resolved_python = resolve_pkgname('python', vinca_conf, distro)
    output['requirements']['run'].extend(resolved_python)
    output['requirements']['host'].extend(resolved_python)
    if pkg.get_build_type() in ['cmake', 'catkin']:
        output['script'] = 'bld_catkin.bat'
    elif pkg.get_build_type() in ['ament_cmake']:
        output['script'] = 'bld_ament_cmake.bat'
    elif pkg.get_build_type() in ['ament_python']:
        output['script'] = 'bld_ament_python.bat'
        resolved_setuptools = resolve_pkgname('setuptools', vinca_conf, distro)
        output['requirements']['host'].extend(resolved_setuptools)
    else:
        return None
    build_deps = pkg.build_depends
    build_deps += pkg.buildtool_depends
    build_deps += pkg.build_export_depends
    build_deps += pkg.buildtool_export_depends
    build_deps += pkg.test_depends
    build_deps = [d.name for d in build_deps if d.evaluated_condition]
    build_deps = set(build_deps)

    for dep in build_deps:
        resolved_dep = resolve_pkgname(dep, vinca_conf, distro)
        if not resolved_dep:
            unsatisfied_deps.add(dep)
            continue
        output['requirements']['host'].extend(resolved_dep)

    run_deps = pkg.run_depends
    run_deps += pkg.exec_depends
    run_deps += pkg.build_export_depends
    run_deps += pkg.buildtool_export_depends
    run_deps = [d.name for d in run_deps if d.evaluated_condition]
    run_deps = set(run_deps)

    for dep in run_deps:
        resolved_dep = resolve_pkgname(dep, vinca_conf, distro)
        if not resolved_dep:
            unsatisfied_deps.add(dep)
            continue
        output['requirements']['run'].extend(resolved_dep)

    output['requirements']['run'] = list(set(output['requirements']['run']))
    output['requirements']['host'] = list(set(output['requirements']['host']))
    output['requirements']['run'] = sorted(output['requirements']['run'])
    output['requirements']['host'] = sorted(output['requirements']['host'])

    return output


def generate_outputs(distro, vinca_conf):
    outputs = []
    for pkg_shortname in vinca_conf['_selected_pkgs']:
        output = generate_output(pkg_shortname, vinca_conf, distro)
        if output is not None:
            outputs.append(output)
    return outputs


def generate_source(distro, vinca_conf):
    source = []
    for pkg_shortname in vinca_conf['_selected_pkgs']:
        url, version = distro.get_released_repo(pkg_shortname)
        entry = {}
        entry['git_url'] = url
        entry['git_rev'] = version
        pkg_names = resolve_pkgname(pkg_shortname, vinca_conf, distro)
        if not pkg_names:
            continue
        pkg_name = pkg_names[0]
        entry['folder'] = '%s/src/work' % pkg_name
        patch_path = os.path.join(
            vinca_conf['_patch_dir'], '%s.patch' % pkg_name)
        if os.path.exists(patch_path):
            entry['patches'] = ['%s/%s' % (
                vinca_conf['patch_dir'], '%s.patch' % pkg_name)]
        source.append(entry)
    return source


def get_selected_packages(distro, vinca_conf):
    selected_packages = set()
    skipped_packages = set()
    if vinca_conf['packages_select_by_deps']:
        for i in vinca_conf['packages_select_by_deps']:
            selected_packages = selected_packages.union([i])
            pkgs = distro.get_depends(i)
            selected_packages = selected_packages.union(pkgs)
    if vinca_conf['packages_skip_by_deps']:
        for i in vinca_conf['packages_skip_by_deps']:
            skipped_packages = skipped_packages.union([i])
            pkgs = distro.get_depends(i)
            skipped_packages = skipped_packages.union(pkgs)
    return selected_packages.difference(skipped_packages)


def main():
    global distro
    global unsatisfied_deps
    arguments = parse_command_line(sys.argv)
    base_dir = os.path.abspath(arguments.dir)
    vinca_yaml = os.path.join(base_dir, 'vinca.yaml')
    vinca_conf = read_vinca_yaml(vinca_yaml)
    vinca_conf['_conda_indexes'] = get_conda_index(vinca_conf)

    distro = Distro(vinca_conf['ros_distro'])

    selected_pkgs = get_selected_packages(distro, vinca_conf)
    # print(selected_pkgs)

    vinca_conf['_selected_pkgs'] = selected_pkgs
    source = generate_source(distro, vinca_conf)
    # print(source)

    outputs = generate_outputs(distro, vinca_conf)
    # print(outputs)

    write_recipe(source, outputs)
    print(unsatisfied_deps)

    from .template import generate_bld_ament_cmake
    from .template import generate_bld_ament_python
    from .template import generate_bld_catkin
    from .template import generate_activate_hook
    generate_bld_ament_cmake()
    generate_bld_ament_python()
    generate_bld_catkin()
    generate_activate_hook()
    print('build scripts are created successfully.')
