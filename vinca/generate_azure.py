import networkx as nx
import yaml
import re
import glob
import sys, os
import textwrap
import argparse
from distutils.dir_util import copy_tree
import yaml


class folded_unicode(str):
    pass


class literal_unicode(str):
    pass


def folded_unicode_representer(dumper, data):
    return dumper.represent_scalar("tag:yaml.org,2002:str", data, style=">")


def literal_unicode_representer(dumper, data):
    return dumper.represent_scalar("tag:yaml.org,2002:str", data, style="|")


yaml.add_representer(folded_unicode, folded_unicode_representer)
yaml.add_representer(literal_unicode, literal_unicode_representer)


azure_linux_script = literal_unicode("""\
export CI=azure
export GIT_BRANCH=$BUILD_SOURCEBRANCHNAME
export FEEDSTOCK_NAME=$(basename ${BUILD_REPOSITORY_NAME})
.scripts/run_docker_build.sh""")

azure_osx_script = literal_unicode("""\
export CI=azure
export GIT_BRANCH=$BUILD_SOURCEBRANCHNAME
export FEEDSTOCK_NAME=$(basename ${BUILD_REPOSITORY_NAME})
.scripts/build_osx.sh""")

azure_win_script = literal_unicode("""\
SETLOCAL EnableDelayedExpansion

echo %PATH%
set "PATH=C:\\Miniconda\\Scripts;C:\\Miniconda;%PATH%"
echo %PATH%


set "CI=azure"
set "FEEDSTOCK_ROOT=%cd%"
call conda.bat activate base

exit 1

conda config --append channels defaults
conda config --add channels conda-forge
conda config --add channels robostack
conda config --set channel_priority strict

conda remove --force m2-git

(for (%%recipe in %CURRENT_RECIPES%) do (
    echo "BUILDING RECIPE %%recipe"
    cd %FEEDSTOCK_ROOT%\\recipes\\%%recipe\\
    copy %FEEDSTOCK_ROOT%\\conda_build_config.yaml .\\conda_build_config.yaml
    boa build .
))

anaconda -t %ANACONDA_API_TOKEN% upload "C:\\bld\\win-64\\*.tar.bz2" --force
""")

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

    parser.add_argument(
        "-a", "--additional-recipes", action="store_true", help="search for additional_recipes folder?")

    arguments = parser.parse_args(argv[1:])
    global parsed_args
    parsed_args = arguments
    return arguments


def normalize_name(s):
    s = s.replace("-", "_")
    return re.sub("[^a-zA-Z0-9_]+", "", s)


def batch_stages(stages, max_batch_size=5):
    print(stages)
    # this reduces the number of individual builds to try to save some time
    stage_lengths = [len(s) for s in stages]
    merged_stages = []
    curr_stage = []

    def chunks(lst, n):
        """Yield successive n-sized chunks from lst."""
        for i in range(0, len(lst), n):
            yield lst[i:i + n]
    i = 0
    while i < len(stages):
        if stage_lengths[i] < max_batch_size and len(curr_stage) + stage_lengths[i] < max_batch_size:
            # merge with previous stage
            curr_stage += stages[i]
        else:
            if len(curr_stage):
                merged_stages.append([curr_stage])
                curr_stage = []
            if stage_lengths[i] < max_batch_size:
                curr_stage += stages[i]
            else:
                # split this stage into multiple
                merged_stages.append(list(chunks(stages[i], max_batch_size)))
        i += 1
    if len(curr_stage):
        merged_stages.append([curr_stage])
    return merged_stages

import requests

def get_skip_existing(vinca_conf, platform):
    fn = vinca_conf.get("skip_existing")
    repodatas = []
    fns = list(fn)
    for fn in fns:
        selected_bn = None
        if "://" in fn:
            fn += f"{platform}/repodata.json"
            print(f"Fetching repodata: {fn}")
            request = requests.get(fn)

            repodata = request.json()
            repodatas.append(repodata)
        else:
            with open(fn) as fi:
                repodata = json.load(fi)
                repodatas.append(repodata)

    return repodatas

def add_additional_recipes(args):
    additional_recipes_path = os.path.abspath(os.path.join(args.dir, '..', 'additional_recipes'))

    print("Searching additional recipes in ", additional_recipes_path)

    if not os.path.exists(additional_recipes_path):
        return

    with open("vinca.yaml", "r") as vinca_yaml:
        vinca_conf = yaml.safe_load(vinca_yaml)

    existing_packages = {}
    repodatas = get_skip_existing(vinca_conf, args.platform)
    additional_recipes = []

    for recipe_path in glob.glob(additional_recipes_path + '/**/recipe.yaml'):
        with open(recipe_path) as recipe:
            additional_recipe = yaml.safe_load(recipe)
 
        name, version, bnumber = (additional_recipe["package"]["name"], additional_recipe["package"]["version"], additional_recipe["build"]["number"])
        print("Checking if ", name, version, bnumber, " exists")
        skip = False
        for repo in repodatas:
            for key, pkg in repo.get("packages", {}).items():
                if pkg["name"] == name and pkg["version"] == version and pkg["build_number"] == bnumber:
                    skip = True
                    print(f"{name}=={version}=={bnumber} already exists. Skipping.")
                    break

        if not skip:
            print("Adding ", os.path.dirname(recipe_path))
            copy_tree(os.path.dirname(recipe_path), args.dir)


def main():

    args = parse_command_line(sys.argv)

    metas = []
    recipe_names = []

    if args.additional_recipes:
        add_additional_recipes(args)

    if not os.path.exists(args.dir):
        print(f"{args.dir} not found. Not generating a pipeline.")

    all_recipes = glob.glob(os.path.join(args.dir, "**", "*.yaml"))
    for f in all_recipes:
        with open(f) as fi:
            metas.append(yaml.safe_load(fi.read()))

    if len(metas) >= 1:
        requirements = {}

        for pkg in metas:
            requirements[pkg["package"]["name"]] = (
                pkg["requirements"].get("host", []) + pkg["requirements"].get("run", [])
            )

        # sort out requirements that are not built in this run
        for pkg_name, reqs in requirements.items():
            requirements[pkg_name] = [
                r.split()[0] for r in reqs if (isinstance(r, str) and r in reqs)
            ]
        print(requirements)

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
    elif len(metas) == 1:
        fn_wo_yaml = os.path.splitext(os.path.basename(all_recipes[0]))[0]
        stages = [[fn_wo_yaml]]
        requirements = [fn_wo_yaml]
    else:
        stages = []
        requirements = []

    azure_template = {"pool": {"vmImage": "ubuntu-16.04"}}

    azure_stages = []

    stage_names = []

    # filter out packages that we are not actually building
    filtered_stages = []
    for stage in stages:
        filtered = [pkg for pkg in stage if pkg in requirements]
        if len(filtered):
            filtered_stages.append(filtered)

    stages = batch_stages(filtered_stages)

    for i, s in enumerate(stages):
        stage_name = f"stage_{i}"
        stage = {"stage": stage_name, "jobs": []}
        stage_names.append(stage_name)

        for batch in s:
            pkg_jobname = '_'.join([normalize_name(pkg) for pkg in batch])
            stage["jobs"].append(
                {
                    "job": pkg_jobname,
                    "steps": [
                        {
                            "script": azure_linux_script,
                            "env": {
                                "ANACONDA_API_TOKEN": "$(ANACONDA_API_TOKEN)",
                                "CURRENT_RECIPES": f"{' '.join([pkg for pkg in batch])}",
                                "DOCKER_IMAGE": "condaforge/linux-anvil-comp7",
                            },
                            "displayName": f"Build {' '.join([pkg for pkg in batch])}",
                        }
                    ],
                }
            )

        if len(stage["jobs"]) != 0:
            # all packages skipped ...
            azure_stages.append(stage)

    azure_template["trigger"] = [args.trigger_branch]
    azure_template["pr"] = "none"
    if azure_stages:
        azure_template["stages"] = azure_stages

    if args.platform == "linux-64" and len(azure_stages):
        with open("linux.yml", "w") as fo:
            fo.write(yaml.dump(azure_template, sort_keys=False))

    azure_template = {"pool": {"vmImage": "macOS-10.15"}}

    azure_stages = []

    stage_names = []
    for i, s in enumerate(stages):
        stage_name = f"stage_{i}"
        stage = {"stage": stage_name, "jobs": []}
        stage_names.append(stage_name)

        for batch in s:
            pkg_jobname = '_'.join([normalize_name(pkg) for pkg in batch])
            stage["jobs"].append(
                {
                    "job": pkg_jobname,
                    "steps": [
                        {
                            "script": azure_osx_script,
                            "env": {
                                "ANACONDA_API_TOKEN": "$(ANACONDA_API_TOKEN)",
                                "CURRENT_RECIPES": f"{' '.join([pkg for pkg in batch])}"
                            },
                            "displayName": f"Build {' '.join([pkg for pkg in batch])}",
                        }
                    ],
                }
            )

        if len(stage["jobs"]) != 0:
            # all packages skipped ...
            azure_stages.append(stage)

    azure_template["trigger"] = [args.trigger_branch]
    azure_template["pr"] = "none"
    if azure_stages:
        azure_template["stages"] = azure_stages

    if args.platform.startswith("osx") and len(azure_stages):
        with open("osx.yml", "w") as fo:
            fo.write(yaml.dump(azure_template, sort_keys=False))

    # Build aarch64 pipeline
    azure_template = {
        "pool": {
            "name": "Default",
            "demands": ["Agent.OS -equals linux", "Agent.OSArchitecture -equals ARM64"],
        }
    }

    azure_stages = []

    stage_names = []
    for i, s in enumerate(stages):
        stage_name = f"stage_{i}"
        stage = {"stage": stage_name, "jobs": []}
        stage_names.append(stage_name)

        for batch in s:
            pkg_jobname = '_'.join([normalize_name(pkg) for pkg in batch])
            stage["jobs"].append(
                {
                    "job": pkg_jobname,
                    "steps": [
                        {
                            "script": azure_linux_script,
                            "env": {
                                "ANACONDA_API_TOKEN": "$(ANACONDA_API_TOKEN)",
                                "CURRENT_RECIPES": f"{' '.join([pkg for pkg in batch])}",
                                "DOCKER_IMAGE": "condaforge/linux-anvil-aarch64",
                            },
                            "displayName": f"Build {' '.join([pkg for pkg in batch])}",
                        }
                    ],
                }
            )

        if len(stage["jobs"]) != 0:
            # all packages skipped ...
            azure_stages.append(stage)

    azure_template["trigger"] = [args.trigger_branch]
    azure_template["pr"] = "none"
    if azure_stages:
        azure_template["stages"] = azure_stages

    if args.platform == "linux-aarch64" and len(azure_stages):
        with open("linux_aarch64.yml", "w") as fo:
            fo.write(yaml.dump(azure_template, sort_keys=False))

    # windows
    azure_template = {"pool": {"vmImage": "vs2017-win2016"}}

    azure_stages = []

    stage_names = []
    for i, s in enumerate(stages):
        stage_name = f"stage_{i}"
        stage = {"stage": stage_name, "jobs": []}
        stage_names.append(stage_name)

        for batch in s:
            pkg_jobname = '_'.join([normalize_name(pkg) for pkg in batch])
            stage["jobs"].append(
                {
                    "job": pkg_jobname,
                    "variables": {"CONDA_BLD_PATH": "C:\\\\bld\\\\"},
                    "steps": [
                        {
                            "task": "CondaEnvironment@1",
                            "inputs": {
                                # "packageSpecs": "python=3.6 dataclasses conda-build conda conda-forge::conda-forge-ci-setup=3 pip boa quetz-client",
                                # "installOptions": "-c conda-forge",
                                # "updateConda": True,
                            },
                            "displayName": "Install conda-build, boa and activate environment",
                        },
                        {
                            "script": literal_unicode(textwrap.dedent("""\
                                set "CI=azure"
                                set "PYTHONUNBUFFERED=1"
                                call conda.bat activate base
                                run_conda_forge_build_setup""")),
                            "displayName": "conda-forge build setup",
                        },
                        {
                            "script": azure_win_script,
                            "env": {
                                "ANACONDA_API_TOKEN": "$(ANACONDA_API_TOKEN)",
                                "CURRENT_RECIPES": f"{' '.join([pkg for pkg in batch])}",
                                "PYTHONUNBUFFERED": 1,
                            },
                            "displayName": f"Build {' '.join([pkg for pkg in batch])}",
                        },
                    ],
                }
            )

        if len(stage["jobs"]) != 0:
            # all packages skipped ...
            azure_stages.append(stage)

    azure_template["trigger"] = [args.trigger_branch]
    azure_template["pr"] = "none"
    if azure_stages:
        azure_template["stages"] = azure_stages

    if args.platform.startswith("win") and len(azure_stages):
        with open("win.yml", "w") as fo:
            fo.write(yaml.dump(azure_template, sort_keys=False))
