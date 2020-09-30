import networkx as nx
import yaml
import re
import glob
import sys, os
import textwrap
import argparse
from collections import OrderedDict

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

# def setup_yaml():
#   """ https://stackoverflow.com/a/8661021 """
#   represent_dict_order = lambda self, data:  self.represent_mapping('tag:yaml.org,2002:map', data.items())
#   yaml.add_representer(OrderedDict, represent_dict_order)
# setup_yaml()

# the stages

"""
# use the official gcc image, based on debian
# can use verions as well, like gcc:5.2
# see https://hub.docker.com/_/gcc/
image: ubuntu:20.04

build:
  stage: build
  script:
    - echo "Hello"
"""


# def str_presenter(dumper, data):
#   if len(data.splitlines()) > 1:  # check for multiline string
#     return dumper.represent_scalar('tag:yaml.org,2002:str', data, style='|')
#   return dumper.represent_scalar('tag:yaml.org,2002:str', data)

# yaml.add_representer(str, str_presenter)

azure_linux_script = """export CI=azure
export GIT_BRANCH=$BUILD_SOURCEBRANCHNAME
export FEEDSTOCK_NAME=$(basename ${BUILD_REPOSITORY_NAME})
.scripts/run_docker_build.sh"""

azure_osx_script = """export CI=azure
export GIT_BRANCH=$BUILD_SOURCEBRANCHNAME
export FEEDSTOCK_NAME=$(basename ${BUILD_REPOSITORY_NAME})
.scripts/build_osx.sh"""

azure_win_script = """
set "CI=azure"
call activate base

conda config --append channels defaults
conda config --add channels conda-forge
conda config --add channels robostack
conda config --set channel_priority strict

conda remove --force m2-git

cp recipes/%CURRENT_BUILD_PKG_NAME%.yaml ./recipe.yaml

boa build .

anaconda -t %ANACONDA_API_TOKEN% upload "C:\\bld\\win-64\\*.tar.bz2" --force
"""

parsed_args = None


def parse_command_line(argv):
    parser = argparse.ArgumentParser(
        description="Conda recipe Azure pipeline generator for ROS packages"
    )

    default_dir = "./recipes"
    parser.add_argument(
        "-d",
        "--dir",
        dest="dir",
        default=default_dir,
        help="The recipes directory to process (default: {}).".format(default_dir),
    )

    parser.add_argument(
        "-t", "--trigger-branch", dest="trigger_branch", help="Trigger branch for Azure"
    )

    parser.add_argument(
        "-p",
        "--platform",
        dest="platform",
        default="linux-64",
        help="Platform to emit build pipeline for",
    )

    arguments = parser.parse_args(argv[1:])
    global parsed_args
    parsed_args = arguments
    return arguments


def normalize_name(s):
    s = s.replace("-", "_")
    return re.sub("[^a-zA-Z0-9_]+", "", s)


def main():

    args = parse_command_line(sys.argv)

    metas = []
    recipe_names = []

    all_recipes = glob.glob(os.path.join(args.dir, "*.yaml"))
    for f in all_recipes:
        with open(f) as fi:
            metas.append(yaml.load(fi.read(), Loader=Loader))

    if len(metas) > 1:
        requirements = {}

        for pkg in metas:
            requirements[pkg["package"]["name"]] = (
                pkg["requirements"]["host"] + pkg["requirements"]["run"]
            )

        # sort out requirements that are not built in this run

        for pkg_name, reqs in requirements.items():
            requirements[pkg_name] = [
                r for r in reqs if (isinstance(r, str) and r in reqs)
            ]
        # print(requirements)

        G = nx.DiGraph()
        for pkg, reqs in requirements.items():
            G.add_node(pkg)
            for r in reqs:
                if r.startswith("ros-"):
                    G.add_edge(pkg, r)

        # import matplotlib.pyplot as plt
        # nx.draw(G, with_labels=True, font_weight='bold')
        # plt.show()

        tg = list(reversed(list(nx.topological_sort(G))))

        stages = []
        current_stage = []
        for pkg in tg:
            reqs = requirements.get(pkg, [])
            sort_in_stage = 0
            for r in reqs:
                # sort up the stages, until first stage found where all requirements are fulfilled.
                for sidx, stage in enumerate(stages):
                    if r in stages[sidx]:
                        sort_in_stage = max(sidx + 1, sort_in_stage)

                # if r in current_stage:
                # stages.append(current_stage)
                # current_stage = []
            if sort_in_stage >= len(stages):
                stages.append([pkg])
            else:
                stages[sort_in_stage].append(pkg)
            # current_stage.append(pkg)

        if len(current_stage):
            stages.append(current_stage)
    else:
        fn_wo_yaml = os.path.splitext(os.path.basename(all_recipes[0]))[0]
        stages = [[fn_wo_yaml]]
        requirements = [fn_wo_yaml]
    # print(stages)

    azure_template = {
        # 'image': 'condaforge/linux-anvil-cos7-x86_64'
    }

    azure_template = {"pool": {"vmImage": "ubuntu-16.04"}}

    azure_stages = []

    stage_names = []
    for i, s in enumerate(stages):
        stage_name = f"stage_{i}"
        stage = {"stage": stage_name, "jobs": []}
        stage_names.append(stage_name)

        for pkg in s:
            # print(pkg)
            if pkg not in requirements:
                continue

            pkg_jobname = normalize_name(pkg)
            stage["jobs"].append(
                {
                    "job": pkg_jobname,
                    "steps": [
                        {
                            # 'script': '''.scripts/build_linux.sh''',
                            "script": azure_linux_script,
                            "env": {
                                "ANACONDA_API_TOKEN": "$(ANACONDA_API_TOKEN)",
                                "CURRENT_BUILD_PKG_NAME": pkg,
                            },
                            "displayName": f"Build {pkg}",
                        }
                    ],
                }
            )

        if len(stage["jobs"]) != 0:
            # all packages skipped ...
            azure_stages.append(stage)

    azure_template["trigger"] = [args.trigger_branch]
    azure_template["pr"] = "none"
    azure_template["stages"] = azure_stages

    if args.platform.startswith("linux"):
        with open("linux.yml", "w") as fo:
            fo.write(yaml.dump(azure_template, Dumper=Dumper, sort_keys=False))

    azure_template = {"pool": {"vmImage": "macOS-10.15"}}

    azure_stages = []

    stage_names = []
    for i, s in enumerate(stages):
        stage_name = f"stage_{i}"
        stage = {"stage": stage_name, "jobs": []}
        stage_names.append(stage_name)

        for pkg in s:
            if pkg not in requirements:
                continue

            pkg_jobname = normalize_name(pkg)
            stage["jobs"].append(
                {
                    "job": pkg_jobname,
                    "steps": [
                        {
                            # 'script': '''.scripts/build_linux.sh''',
                            "script": azure_osx_script,
                            "env": {
                                "ANACONDA_API_TOKEN": "$(ANACONDA_API_TOKEN)",
                                "CURRENT_BUILD_PKG_NAME": pkg,
                            },
                            "displayName": f"Build {pkg}",
                        }
                    ],
                }
            )

        if len(stage["jobs"]) != 0:
            # all packages skipped ...
            azure_stages.append(stage)

    azure_template["trigger"] = [args.trigger_branch]
    azure_template["pr"] = "none"
    azure_template["stages"] = azure_stages

    if args.platform.startswith("osx"):
        with open("osx.yml", "w") as fo:
            fo.write(yaml.dump(azure_template, Dumper=Dumper, sort_keys=False))

    # windows
    azure_template = {"pool": {"vmImage": "vs2017-win2016"}}

    azure_stages = []

    stage_names = []
    for i, s in enumerate(stages):
        stage_name = f"stage_{i}"
        stage = {"stage": stage_name, "jobs": []}
        stage_names.append(stage_name)

        for pkg in s:
            if pkg not in requirements:
                continue

            pkg_jobname = normalize_name(pkg)
            stage["jobs"].append(
                {
                    "job": pkg_jobname,
                    "variables": {"CONDA_BLD_PATH": "C:\\\\bld\\\\"},
                    "steps": [
                        {
                            "task": "CondaEnvironment@1",
                            "inputs": {
                                "packageSpecs": "python=3.6 dataclasses conda-build conda conda-forge::conda-forge-ci-setup=3 pip boa quetz-client",
                                "installOptions": "-c conda-forge",
                                "updateConda": True,
                            },
                            "displayName": "Install conda-build, boa and activate environment",
                        },
                        # {
                        #   'script': "rmdir C:\\cygwin /s /q",
                        #   'displayName': 'Remove cygwin to make git work',
                        #   'continueOnError': True
                        # },
                        {
                            "script": textwrap.dedent(
                                """
                        set "CI=azure"
                        call activate base
                        run_conda_forge_build_setup"""
                            ),
                            "displayName": "conda-forge build setup",
                        },
                        {
                            "script": azure_win_script,
                            "env": {
                                "ANACONDA_API_TOKEN": "$(ANACONDA_API_TOKEN)",
                                "CURRENT_BUILD_PKG_NAME": pkg,
                                "PYTHONUNBUFFERED": 1,
                            },
                            "displayName": f"Build {pkg}",
                        },
                    ],
                }
            )

        if len(stage["jobs"]) != 0:
            # all packages skipped ...
            azure_stages.append(stage)

    azure_template["trigger"] = [args.trigger_branch]
    azure_template["pr"] = "none"
    azure_template["stages"] = azure_stages

    if args.platform.startswith("win"):
        with open("win.yml", "w") as fo:
            fo.write(yaml.dump(azure_template, Dumper=Dumper, sort_keys=False))
