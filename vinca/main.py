#!/usr/bin/env python

import argparse
import catkin_pkg
import sys
import subprocess
import os
from .repos import get_repos
from vinca import __version__
from .resolve import get_conda_index
from .resolve import resolve_pkgname
from .template import write_recipe

unsatisfied_deps = set()


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
    vinca_conf['repos'] = os.path.abspath(vinca_conf['repos'])

    conda_index = []
    for i in vinca_conf['conda_index']:
        conda_index.append(os.path.abspath(i))
    vinca_conf['conda_index'] = conda_index
    vinca_conf['patch_dir'] = os.path.abspath(vinca_conf['patch_dir'])
    return vinca_conf


def generate_output(pkg_shortname, vinca_conf, rospack):
    if pkg_shortname not in vinca_conf['_selected_pkgs']:
        return None
    manifest = rospack.get_manifest(pkg_shortname)
    output = {
        'name': resolve_pkgname(pkg_shortname, vinca_conf)[0],
        'version': manifest.version,
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
    package_uri = rospack.get_path(pkg_shortname)
    pkg = catkin_pkg.package.parse_package(
        os.path.join(package_uri, 'package.xml'))
    pkg.evaluate_conditions(os.environ)
    resolved_python = resolve_pkgname('python', vinca_conf)
    output['requirements']['run'].extend(resolved_python)
    output['requirements']['host'].extend(resolved_python)
    if pkg.get_build_type() in ['cmake', 'catkin', 'ament_cmake']:
        output['script'] = 'bld_cmake.bat'
    elif pkg.get_build_type() in ['ament_python']:
        output['script'] = 'bld_python.bat'
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
        resolved_dep = resolve_pkgname(dep, vinca_conf)
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
        resolved_dep = resolve_pkgname(dep, vinca_conf)
        if not resolved_dep:
            unsatisfied_deps.add(dep)
            continue
        output['requirements']['run'].extend(resolved_dep)

    output['requirements']['run'] = list(set(output['requirements']['run']))
    output['requirements']['host'] = list(set(output['requirements']['host']))
    output['requirements']['run'] = sorted(output['requirements']['run'])
    output['requirements']['host'] = sorted(output['requirements']['host'])

    return output


def generate_outputs(base_dir, vinca_conf):
    import rospkg
    outputs = []
    rospack = rospkg.RosPack([base_dir])
    for pkg_shortname in rospack.list():
        output = generate_output(pkg_shortname, vinca_conf, rospack)
        if output is not None:
            outputs.append(output)
    return outputs


def generate_source(repos, base_dir, vinca_conf):
    import rospkg
    import copy
    source = []
    for path, repo in repos.items():
        entry = {}
        path_root = os.path.abspath(os.path.join(base_dir, path))
        rospack = rospkg.RosPack([path_root])
        if repo['type'] == 'git':
            entry['git_url'] = repo['url']
            entry['git_rev'] = repo['version']
        for pkg_shortname in rospack.list():
            if pkg_shortname not in vinca_conf['_selected_pkgs']:
                continue
            local_entry = copy.deepcopy(entry)
            pkg_name = resolve_pkgname(pkg_shortname, vinca_conf)[0]
            local_entry['folder'] = '%s/src/work' % pkg_name
            source.append(local_entry)
    return source


def onerror(func, path, exc_info):
    """
    Error handler for ``shutil.rmtree``.

    If the error is due to an access error (read only file)
    it attempts to add write permission and then retries.

    If the error is for another reason it re-raises the error.

    Usage : ``shutil.rmtree(path, onerror=onerror)``
    """
    import stat
    if not os.access(path, os.W_OK):
        # Is the error an access error ?
        os.chmod(path, stat.S_IWUSR)
        func(path)
    else:
        raise


def get_selected_packages(base_dir, vinca_conf):
    import rospkg
    rospack = rospkg.RosPack([base_dir])

    selected_packages = set()
    if vinca_conf['packages_select_by_deps']:
        for i in vinca_conf['packages_select_by_deps']:
            pkgs = rospack.get_depends(i)
            selected_packages = selected_packages.union(pkgs)
    if vinca_conf['packages_skip_by_deps']:
        for i in vinca_conf['packages_skip_by_deps']:
            pkgs = rospack.get_depends(i)
            selected_packages = selected_packages.difference(pkgs)
    return selected_packages


def main():
    arguments = parse_command_line(sys.argv)
    base_dir = os.path.abspath(arguments.dir)
    vinca_yaml = os.path.join(base_dir, 'vinca.yaml')
    vinca_conf = read_vinca_yaml(vinca_yaml)
    vinca_conf['_conda_indexes'] = get_conda_index(vinca_conf)

    os.environ['ROS_PYTHON_VERSION'] = '{0}'.format(
        vinca_conf['ros_python_version'])
    os.environ['ROS_DISTRO'] = '{0}'.format(
        vinca_conf['ros_python_version'])
    if 'ROS_ROOT' in os.environ:
        os.environ.pop('ROS_ROOT')
    if 'ROS_PACKAGE_PATH' in os.environ:
        os.environ.pop('ROS_PACKAGE_PATH')
    # print(vinca_conf)

    repos = get_repos(vinca_conf['repos'])
    # print(repos)

    import tempfile
    tmpdirname = tempfile.mkdtemp()
    print('created temporary directory', tmpdirname)
    base_src = os.path.join(tmpdirname, 'src')
    os.mkdir(base_src)
    subprocess.check_call(['vcs', 'import', base_src],
                          stdin=open(vinca_conf['repos'], 'r'))

    selected_pkgs = get_selected_packages(base_src, vinca_conf)
    vinca_conf['_selected_pkgs'] = selected_pkgs
    source = generate_source(repos, base_src, vinca_conf)
    # print(source)

    outputs = generate_outputs(base_src, vinca_conf)
    # print(outputs)

    write_recipe(source, outputs)
    print(unsatisfied_deps)

    import shutil
    shutil.rmtree(tmpdirname, onerror=onerror)
    print('meta.yaml is created successfully.')

    from .template import generate_bld_cmake
    from .template import generate_bld_python
    generate_bld_cmake()
    generate_bld_python()
    print('bld_cmake.bat and bld_python.bat are created successfully.')
