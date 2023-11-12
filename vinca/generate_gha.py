import networkx as nx
import yaml
import re
import glob
import sys
import os
import argparse
import pkg_resources
from distutils.dir_util import copy_tree

from rich import print

from vinca.utils import get_repodata, NoAliasDumper
from vinca.utils import literal_unicode as lu
from vinca.distro import Distro
from vinca.main import (
    get_selected_packages,
    generate_outputs,
    read_vinca_yaml,
    get_conda_subdir,
)
from vinca import config


def read_azure_script(fn):
    template_in = pkg_resources.resource_filename("vinca", f"azure_templates/{fn}")
    with open(template_in, "r") as fi:
        return fi.read()


azure_linux_script = lu(read_azure_script("linux.sh"))
azure_osx_script = lu(read_azure_script("osx_64.sh"))
azure_osx_arm64_script = lu(read_azure_script("osx_arm64.sh"))
azure_win_preconfig_script = lu(read_azure_script("win_preconfig.bat"))
azure_win_script = lu(read_azure_script("win_build.bat"))


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
        "-a",
        "--additional-recipes",
        action="store_true",
        help="search for additional_recipes folder?",
    )
    
    parser.add_argument(
        "-b",
        "--batch_size",
        dest="batch_size",
        default=5,
        type=int,
        help="How many packages to build at most per stage",
    )

    arguments = parser.parse_args(argv[1:])
    config.parsed_args = arguments
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
            yield lst[i : i + n]

    i = 0
    while i < len(stages):
        for build_individually_pkg in build_individually:
            if build_individually_pkg in stages[i]:
                merged_stages.append([[build_individually_pkg]])
                stages[i].remove(build_individually_pkg)

        if (
            stage_lengths[i] < max_batch_size
            and len(curr_stage) + stage_lengths[i] < max_batch_size
        ):
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


def get_skip_existing(vinca_conf, platform):
    fn = vinca_conf.get("skip_existing")
    repodatas = []
    if fn is not None:
        fns = list(fn)
    else:
        fns = []

    for fn in fns:
        print(f"Fetching repodata: {fn}")
        repodata = get_repodata(fn, platform)
        repodatas.append(repodata)

    return repodatas


def get_all_ancestors(graph, node):
    ancestors = set()
    visited = set()
    current_node = node

    while True:
        a = {
            a
            for a in graph.get(node, [])
            if a.startswith("ros-") or a.startswith("ros2")
        }
        if not graph.get(node):
            print(f"[yellow]{node} not found")

        ancestors |= a
        visited.add(current_node)

        if len(ancestors - visited) == 0:
            print(f"Returning all ancestors for {node} : {ancestors}")
            return ancestors
        else:
            current_node = list(ancestors - visited)[0]


def add_additional_recipes(args):
    additional_recipes_path = os.path.abspath(
        os.path.join(args.dir, "..", "additional_recipes")
    )

    print("Searching additional recipes in ", additional_recipes_path)

    if not os.path.exists(additional_recipes_path):
        return

    with open("vinca.yaml", "r") as vinca_yaml:
        vinca_conf = yaml.safe_load(vinca_yaml)

    repodatas = get_skip_existing(vinca_conf, args.platform)

    additional_recipes = []
    for recipe_path in glob.glob(additional_recipes_path + "/**/recipe.yaml"):
        with open(recipe_path) as recipe:
            additional_recipe = yaml.safe_load(recipe)

        name, version, bnumber = (
            additional_recipe["package"]["name"],
            additional_recipe["package"]["version"],
            additional_recipe["build"]["number"],
        )
        print("Checking if ", name, version, bnumber, " exists")
        skip = False
        for repo in repodatas:
            for _, pkg in repo.get("packages", {}).items():
                if (
                    pkg["name"] == name
                    and pkg["version"] == version
                    and pkg["build_number"] == bnumber
                ):
                    skip = True
                    print(f"{name}=={version}=={bnumber} already exists. Skipping.")
                    break

        if not skip:
            print("Adding ", os.path.dirname(recipe_path))
            goal_folder = os.path.join(args.dir, name)
            os.makedirs(goal_folder, exist_ok=True)
            copy_tree(os.path.dirname(recipe_path), goal_folder)
            additional_recipes.append(additional_recipe)

    return additional_recipes


# on:
#   pull_request:
#     paths:
#       - '*.yaml'


def dump_for_gha(doc, f):
    s = yaml.dump(doc, sort_keys=False, Dumper=NoAliasDumper)
    s = s.replace("'on':", "on:")
    with open(f, "w") as fo:
        fo.write(s)


def get_stage_name(batch):
    stage_name = []
    for pkg in batch:
        if len(pkg.split("-")) > 2:
            stage_name.append("-".join(pkg.split("-")[2:]))
        else:
            stage_name.append(pkg)
    return " ".join(stage_name)


def build_linux_pipeline(
    stages,
    trigger_branch,
    script=azure_linux_script,
    azure_template=None,
    docker_image=None,
    runs_on=None,
    outfile="linux.yml",
):

    blurb = {"jobs": {}, "name": "build_linux"}

    if runs_on is None:
        runs_on = "ubuntu-latest"

    # Build Linux pipeline
    if azure_template is None:
        azure_template = blurb

    if docker_image is None:
        docker_image = "condaforge/linux-anvil-cos7-x86_64"

    jobs = []
    job_names = []
    prev_batch_keys = []

    for i, s in enumerate(stages):
        stage_name = f"stage_{i}"
        batch_keys = []
        for batch in s:
            batch_key = f"{stage_name}_job_{len(azure_template['jobs'])}"
            batch_keys.append(batch_key)

            pretty_stage_name = get_stage_name(batch)
            azure_template["jobs"][batch_key] = {
                "name": pretty_stage_name,
                "runs-on": runs_on,
                "strategy": {"fail-fast": False},
                "needs": prev_batch_keys,
                "steps": [
                    {"name": "Checkout code", "uses": "actions/checkout@v3"},
                    {
                        "name": f"Build {' '.join([pkg for pkg in batch])}",
                        "env": {
                            "ANACONDA_API_TOKEN": "${{ secrets.ANACONDA_API_TOKEN }}",
                            "CURRENT_RECIPES": f"{' '.join([pkg for pkg in batch])}",
                            "DOCKER_IMAGE": docker_image,
                        },
                        "run": script,
                    },
                ],
            }

        prev_batch_keys = batch_keys

    if len(azure_template.get("jobs", [])) == 0:
        return

    azure_template["on"] = {"push": {"branches": [trigger_branch]}}

    dump_for_gha(azure_template, outfile)


def build_osx_pipeline(
    stages,
    trigger_branch,
    vm_imagename="macos-11",
    outfile="osx.yml",
    azure_template=None,
    script=azure_osx_script,
):
    # Build OSX pipeline
    blurb = {"jobs": {}, "name": "build_osx"}

    # Build Linux pipeline
    if azure_template is None:
        azure_template = blurb

    jobs = []
    job_names = []
    prev_batch_keys = []
    for i, s in enumerate(stages):
        stage_name = f"stage_{i}"
        batch_keys = []
        for batch in s:
            batch_key = f"{stage_name}_job_{len(azure_template['jobs'])}"
            batch_keys.append(batch_key)

            pretty_stage_name = get_stage_name(batch)
            azure_template["jobs"][batch_key] = {
                "name": pretty_stage_name,
                "runs-on": vm_imagename,
                "strategy": {"fail-fast": False},
                "needs": prev_batch_keys,
                "steps": [
                    {"name": "Checkout code", "uses": "actions/checkout@v3"},
                    {
                        "name": f"Build {' '.join([pkg for pkg in batch])}",
                        "env": {
                            "ANACONDA_API_TOKEN": "${{ secrets.ANACONDA_API_TOKEN }}",
                            "CURRENT_RECIPES": f"{' '.join([pkg for pkg in batch])}",
                        },
                        "run": script,
                    },
                ],
            }

        prev_batch_keys = batch_keys

    if len(azure_template.get("jobs", [])) == 0:
        return

    azure_template["on"] = {"push": {"branches": [trigger_branch]}}

    dump_for_gha(azure_template, outfile)


def build_win_pipeline(stages, trigger_branch, outfile="win.yml", azure_template=None):

    vm_imagename = "windows-2019"
    # Build Win pipeline
    blurb = {"jobs": {}, "name": "build_win"}

    if azure_template is None:
        azure_template = blurb

    script = azure_win_script

    # overwrite with what we're finding in the repo!
    if os.path.exists(".scripts/build_win.bat"):
        with open(".scripts/build_win.bat", "r") as fi:
            script = lu(fi.read())

    jobs = []
    job_names = []
    prev_batch_keys = []
    for i, s in enumerate(stages):
        stage_name = f"stage_{i}"
        batch_keys = []
        for batch in s:
            batch_key = f"{stage_name}_job_{len(azure_template['jobs'])}"
            batch_keys.append(batch_key)

            pretty_stage_name = get_stage_name(batch)
            azure_template["jobs"][batch_key] = {
                "name": pretty_stage_name,
                "runs-on": vm_imagename,
                "strategy": {"fail-fast": False},
                "needs": prev_batch_keys,
                "env": {"CONDA_BLD_PATH": "C:\\\\bld\\\\"},
                "steps": [
                    {"name": "Checkout code", "uses": "actions/checkout@v3"},
                    {
                        "uses": "conda-incubator/setup-miniconda@v2",
                        "with": {
                            "channels": "conda-forge",
                            "miniforge-variant": "Mambaforge",
                            "miniforge-version": "latest",
                            "use-mamba": "true",
                            "channel-priority": "true",
                        },
                    },
                    {
                        "run": "conda install -c conda-forge -n base --yes --quiet conda-build=3.25 pip mamba ruamel.yaml anaconda-client",
                        "name": "Install conda-build, boa and activate environment",
                    },
                    {
                        "shell": "cmd",
                        "run": azure_win_preconfig_script,
                        "name": "conda-forge build setup",
                    },
                    {
                        "shell": "cmd",
                        "run": script,
                        "env": {
                            "ANACONDA_API_TOKEN": "${{ secrets.ANACONDA_API_TOKEN }}",
                            "CURRENT_RECIPES": f"{' '.join([pkg for pkg in batch])}",
                            "PYTHONUNBUFFERED": 1,
                        },
                        "name": f"Build {' '.join([pkg for pkg in batch])}",
                    },
                ],
            }

        prev_batch_keys = batch_keys

    if len(azure_template.get("jobs", [])) == 0:
        return

    azure_template["on"] = {"push": {"branches": [trigger_branch]}}

    dump_for_gha(azure_template, outfile)


def get_full_tree():
    recipes_dir = config.parsed_args.dir

    vinca_yaml = os.path.join(os.path.dirname(recipes_dir), "vinca.yaml")

    temp_vinca_conf = read_vinca_yaml(vinca_yaml)
    temp_vinca_conf["build_all"] = True
    temp_vinca_conf["skip_built_packages"] = []
    config.selected_platform = get_conda_subdir()

    python_version = temp_vinca_conf.get("python_version", None)
    distro = Distro(temp_vinca_conf["ros_distro"], python_version)

    all_packages = get_selected_packages(distro, temp_vinca_conf)
    temp_vinca_conf["_selected_pkgs"] = all_packages

    all_outputs = generate_outputs(distro, temp_vinca_conf)
    return all_outputs


def main():

    args = parse_command_line(sys.argv)

    full_tree = get_full_tree()

    metas = []

    additional_recipes = []
    if args.additional_recipes:
        additional_recipes = add_additional_recipes(args)

    if not os.path.exists(args.dir):
        print(f"{args.dir} not found. Not generating a pipeline.")

    all_recipes = glob.glob(os.path.join(args.dir, "**", "*.yaml"))
    for f in all_recipes:
        with open(f) as fi:
            metas.append(yaml.safe_load(fi.read()))

    if len(metas) >= 1:
        requirements = {}

        for pkg in full_tree + additional_recipes:
            requirements[pkg["package"]["name"]] = pkg["requirements"].get(
                "host", []
            ) + pkg["requirements"].get("run", [])

        # sort out requirements that are not built in this run
        for pkg_name, reqs in requirements.items():
            requirements[pkg_name] = [
                r.split()[0] for r in reqs if (isinstance(r, str) and r in reqs)
            ]

        G = nx.DiGraph()
        for pkg, reqs in requirements.items():
            G.add_node(pkg)
            for r in reqs:
                if r.startswith("ros-") or r.startswith("ros2-"):
                    G.add_edge(pkg, r)

        # print(requirements)
        # import matplotlib.pyplot as plt
        # nx.draw(G, with_labels=True, font_weight='bold')
        # plt.show()

        tg = list(reversed(list(nx.topological_sort(G))))

        names_to_build = {pkg["package"]["name"] for pkg in metas}
        print("Names to build: ", names_to_build)
        tg_slimmed = [el for el in tg if el in names_to_build]

        stages = []
        current_stage = []
        for pkg in tg_slimmed:
            reqs = get_all_ancestors(requirements, pkg)

            sort_in_stage = 0
            for r in reqs:
                # sort up the stages, until first stage found where all requirements are fulfilled.
                for sidx, _ in enumerate(stages):
                    if r in stages[sidx]:
                        sort_in_stage = max(sidx + 1, sort_in_stage)

            if sort_in_stage >= len(stages):
                stages.append([pkg])
            else:
                stages[sort_in_stage].append(pkg)

        if len(current_stage):
            stages.append(current_stage)

    elif len(metas) == 1:
        fn_wo_yaml = os.path.splitext(os.path.basename(all_recipes[0]))[0]
        stages = [[fn_wo_yaml]]
        requirements = [fn_wo_yaml]
    else:
        stages = []
        requirements = []

    # filter out packages that we are not actually building
    filtered_stages = []
    for stage in stages:
        filtered = [pkg for pkg in stage if pkg in requirements]
        if len(filtered):
            filtered_stages.append(filtered)

    stages = batch_stages(filtered_stages, args.batch_size)
    print(stages)

    with open("buildorder.txt", "w") as fo:
        order = []
        for stage in filtered_stages:
            for el in stage:
                print(el)
                order.append(el)

        fo.write("\n".join(order))

    if args.platform == "linux-64":
        build_linux_pipeline(stages, args.trigger_branch, outfile="linux.yml")

    if args.platform == "osx-64":
        build_osx_pipeline(
            stages, args.trigger_branch, script=azure_osx_script,
        )

    if args.platform == "osx-arm64":
        build_osx_pipeline(
            stages,
            args.trigger_branch,
            vm_imagename="macOS-11",
            outfile="osx_arm64.yml",
            script=azure_osx_arm64_script,
        )

    if args.platform == "linux-aarch64":
        # Build aarch64 pipeline
        build_linux_pipeline(
            stages,
            args.trigger_branch,
            runs_on="cirun-linux-aarch64--${{ github.run_id }}",
            docker_image="condaforge/linux-anvil-aarch64",
            outfile="linux_aarch64.yml",
        )

    # windows
    if args.platform == "win-64":
        build_win_pipeline(stages, args.trigger_branch, outfile="win.yml")
