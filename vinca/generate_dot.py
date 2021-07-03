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

azure_osx_script = literal_unicode(r"""\
export CI=azure
export GIT_BRANCH=$BUILD_SOURCEBRANCHNAME
export FEEDSTOCK_NAME=$(basename ${BUILD_REPOSITORY_NAME})
.scripts/build_osx.sh""")

azure_osx_arm64_script = literal_unicode(r"""\
export CI=azure
export GIT_BRANCH=$BUILD_SOURCEBRANCHNAME
export FEEDSTOCK_NAME=$(basename ${BUILD_REPOSITORY_NAME})
.scripts/build_osx_arm64.sh""")

azure_win_preconfig_script = literal_unicode("""\
set "CI=azure"
call %CONDA%\\condabin\\conda_hook.bat
call %CONDA%\\condabin\\conda.bat activate base

:: 2 cores available on Appveyor workers: https://www.appveyor.com/docs/build-environment/#build-vm-configurations
:: CPU_COUNT is passed through conda build: https://github.com/conda/conda-build/pull/1149
set CPU_COUNT=2

set PYTHONUNBUFFERED=1

conda config --set show_channel_urls true
conda config --set auto_update_conda false
conda config --set add_pip_as_python_dependency false

call setup_x64

:: Set the conda-build working directory to a smaller path
if "%CONDA_BLD_PATH%" == "" (
    set "CONDA_BLD_PATH=C:\\bld\\"
)

:: Remove some directories from PATH
set "PATH=%PATH:C:\\ProgramData\\Chocolatey\\bin;=%"
set "PATH=%PATH:C:\\Program Files (x86)\\sbt\\bin;=%"
set "PATH=%PATH:C:\\Rust\\.cargo\\bin;=%"
set "PATH=%PATH:C:\\Program Files\\Git\\usr\\bin;=%"
set "PATH=%PATH:C:\\Program Files\\Git\\cmd;=%"
set "PATH=%PATH:C:\\Program Files\\Git\\mingw64\\bin;=%"
set "PATH=%PATH:C:\\Program Files (x86)\\Subversion\\bin;=%"
set "PATH=%PATH:C:\\Program Files\\CMake\\bin;=%"
set "PATH=%PATH:C:\\Program Files\\OpenSSL\\bin;=%"
set "PATH=%PATH:C:\\Strawberry\\c\\bin;=%"
set "PATH=%PATH:C:\\Strawberry\\perl\\bin;=%"
set "PATH=%PATH:C:\\Strawberry\\perl\\site\\bin;=%"
set "PATH=%PATH:c:\\tools\\php;=%"

:: On azure, there are libcrypto*.dll & libssl*.dll under
:: C:\\Windows\\System32, which should not be there (no vendor dlls in windows folder).
:: They would be found before the openssl libs of the conda environment, so we delete them.
if defined CI (
    DEL C:\\Windows\\System32\\libcrypto-1_1-x64.dll || (Echo Ignoring failure to delete C:\\Windows\\System32\\libcrypto-1_1-x64.dll)
    DEL C:\\Windows\\System32\\libssl-1_1-x64.dll || (Echo Ignoring failure to delete C:\\Windows\\System32\\libssl-1_1-x64.dll)
)

:: Make paths like C:\\hostedtoolcache\\windows\\Ruby\\2.5.7\\x64\\bin garbage
set "PATH=%PATH:ostedtoolcache=%"

mkdir "%CONDA%\\etc\\conda\\activate.d"

echo set "CONDA_BLD_PATH=%CONDA_BLD_PATH%"         > "%CONDA%\\etc\\conda\\activate.d\\conda-forge-ci-setup-activate.bat"
echo set "CPU_COUNT=%CPU_COUNT%"                  >> "%CONDA%\\etc\\conda\\activate.d\\conda-forge-ci-setup-activate.bat"
echo set "PYTHONUNBUFFERED=%PYTHONUNBUFFERED%"    >> "%CONDA%\\etc\\conda\\activate.d\\conda-forge-ci-setup-activate.bat"
echo set "PATH=%PATH%"                            >> "%CONDA%\\etc\\conda\\activate.d\\conda-forge-ci-setup-activate.bat"

conda info
conda config --show-sources
conda list --show-channel-urls
""")

azure_win_script = literal_unicode("""\
setlocal EnableExtensions EnableDelayedExpansion
call %CONDA%\\condabin\\conda_hook.bat
call %CONDA%\\condabin\\conda.bat activate base

set "FEEDSTOCK_ROOT=%cd%"

call conda config --append channels defaults
call conda config --add channels conda-forge
call conda config --add channels robostack
call conda config --set channel_priority strict

:: conda remove --force m2-git

C:\\Miniconda\\python.exe -m pip install git+https://github.com/mamba-org/boa.git@master
if errorlevel 1 exit 1

for %%X in (%CURRENT_RECIPES%) do (
    echo "BUILDING RECIPE %%X"
    cd %FEEDSTOCK_ROOT%\\recipes\\%%X\\
    copy %FEEDSTOCK_ROOT%\\conda_build_config.yaml .\\conda_build_config.yaml
    boa build .
    if errorlevel 1 exit 1
)

anaconda -t %ANACONDA_API_TOKEN% upload "C:\\bld\\win-64\\*.tar.bz2" --force
if errorlevel 1 exit 1
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
    with open("vinca.yaml", "r") as vinca_yaml:
        vinca_conf = yaml.safe_load(vinca_yaml)

    # this reduces the number of individual builds to try to save some time
    stage_lengths = [len(s) for s in stages]
    merged_stages = []
    curr_stage = []
    build_individually = vinca_conf.get("build_in_own_azure_stage", [])

    def chunks(lst, n):
        """Yield successive n-sized chunks from lst."""
        for i in range(0, len(lst), n):
            yield lst[i:i + n]
    i = 0
    while i < len(stages):
        for build_individually_pkg in build_individually:
            if build_individually_pkg in stages[i]:
                merged_stages.append([[build_individually_pkg]])
                stages[i].remove(build_individually_pkg)

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
    if fn is not None:
        fns = list(fn)
    else:
        fns = []
    for fn in fns:
        selected_bn = None
        if "://" in fn:
            fn += f"{platform}/repodata.json"
            print(f"Fetching repodata: {fn}")
            request = requests.get(fn)

            repodata = request.json()
            repodatas.append(repodata)
        else:
            import json
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

    repodatas = get_skip_existing(vinca_conf, args.platform)

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
            goal_folder = os.path.join(args.dir, name)
            os.makedirs(goal_folder, exist_ok=True)
            copy_tree(os.path.dirname(recipe_path), goal_folder)


def main():

    args = parse_command_line(sys.argv)

    metas = []

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

        import matplotlib.pyplot as plt
        from networkx.drawing.nx_agraph import write_dot

        nx.draw(G, with_labels=True, font_weight='bold')
        plt.show()

        write_dot(G, "grid.dot")

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
