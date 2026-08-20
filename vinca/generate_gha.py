import argparse
import glob
import os
import sys
from distutils.dir_util import copy_tree
from importlib import resources

import networkx as nx
import yaml
import re
from typing import Any
from rich import print

from vinca import config
from vinca.distro import Distro
from vinca.main import (
    generate_outputs,
    get_conda_subdir,
    get_selected_packages,
    read_vinca_yaml,
)
from vinca.pipeline import batch_stages, get_all_ancestors, get_skip_existing
from vinca.utils import (
    NoAliasDumper,
    add_test_requirements,
    build_requirement_graph,
    extract_dependency_names,
)
from vinca.utils import literal_unicode as lu


# Use the v0 version of setup-pixi by default, which should give you the last major release
DEFAULT_SETUP_PIXI_VERSION = "v0"
DEFAULT_PIXI_VERSION = "latest"


def read_azure_script(fn):
    return (resources.files("vinca") / "azure_templates" / fn).read_text(
        encoding="utf-8"
    )


azure_unix_script = lu(read_azure_script("unix.sh"))
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
            repo_pkgs = repo.get("packages", {})
            repo_pkgs.update(repo.get("packages.conda", {}))
            for _, pkg in repo_pkgs.items():
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


MAX_WORKFLOW_SIZE_BYTES = 500 * 1024


def dump_for_gha(doc, f):
    s = yaml.dump(doc, sort_keys=False, Dumper=NoAliasDumper)
    s = s.replace("'on':", "on:")
    with open(f, "w") as fo:
        fo.write(s)

    workflow_size = os.path.getsize(f)
    if workflow_size > MAX_WORKFLOW_SIZE_BYTES:
        raise RuntimeError(
            f"Generated workflow {f} is {workflow_size / 1024:.1f} KiB, exceeding "
            f"the {MAX_WORKFLOW_SIZE_BYTES / 1024:.0f} KiB limit. Increase "
            "--batch_size to reduce the number of generated jobs."
        )


def get_stage_name(batch):
    legacy_prefix = f"ros-{config.ros_distro}-"
    stage_name = []
    for pkg in batch:
        if pkg.startswith(legacy_prefix):
            stage_name.append(pkg[len(legacy_prefix) :])
        else:
            stage_name.append(pkg)
    return " ".join(stage_name)


def normalize_version(version: str) -> str:
    """Normalize a release while leaving action refs such as commit SHAs intact."""
    version = str(version).strip()
    if not version:
        raise ValueError("Version values must not be empty")

    # Full GitHub Action commit hashes and symbolic refs such as ``latest`` must
    # be passed through verbatim. Only a bare semantic version gets a v prefix.
    if re.fullmatch(r"[0-9a-fA-F]{40}", version):
        return version
    if re.fullmatch(r"v\d+\.\d+\.\d+(?:[-+][0-9A-Za-z.-]+)?", version):
        return version
    if re.fullmatch(r"\d+\.\d+\.\d+(?:[-+][0-9A-Za-z.-]+)?", version):
        return f"v{version}"
    return version


def get_setup_pixi_step(
    setup_pixi_version: str = DEFAULT_SETUP_PIXI_VERSION,
    pixi_version: str = DEFAULT_PIXI_VERSION,
) -> dict[str, Any]:
    return {
        "name": "Setup pixi",
        "uses": f"prefix-dev/setup-pixi@{normalize_version(setup_pixi_version)}",
        "with": {
            "pixi-version": normalize_version(pixi_version),
            "cache": "true",
            "log-level": "v",
            "frozen": "true",
        },
    }


def build_unix_pipeline(
    stages,
    trigger_branch,
    script=azure_unix_script,
    azure_template=None,
    runs_on="ubuntu-latest",
    outfile="linux.yml",
    pipeline_name="build_unix",
    target="",
    setup_pixi_version: str = DEFAULT_SETUP_PIXI_VERSION,
    pixi_version: str = DEFAULT_PIXI_VERSION,
):
    blurb = {"jobs": {}, "name": pipeline_name}

    if azure_template is None:
        azure_template = blurb

    prev_batch_keys = []

    for i, s in enumerate(stages):
        stage_name = f"stage_{i}"
        batch_keys = []
        for batch in s:
            batch_key = f"{stage_name}_job_{len(azure_template['jobs'])}"
            batch_keys.append(batch_key)

            pretty_stage_name = get_stage_name(batch)

            build_env = {
                "ANACONDA_API_TOKEN": "${{ secrets.ANACONDA_API_TOKEN }}",
                "CURRENT_RECIPES": f"{' '.join([pkg for pkg in batch])}",
                "BUILD_TARGET": target,
            }

            steps = [
                {
                    "name": "Checkout code",
                    "uses": "actions/checkout@v7",
                },
                get_setup_pixi_step(setup_pixi_version, pixi_version),
                {
                    "name": f"Build {' '.join([pkg for pkg in batch])}",
                    "env": build_env,
                    "run": script,
                },
            ]

            job = {
                "name": pretty_stage_name,
                "runs-on": runs_on,
                "strategy": {"fail-fast": False},
                "needs": prev_batch_keys,
                "steps": steps,
            }

            job["permissions"] = {
                "id-token": "write",
                "attestations": "write",
            }

            azure_template["jobs"][batch_key] = job

        prev_batch_keys = batch_keys

    if len(azure_template.get("jobs", [])) == 0:
        return

    azure_template["on"] = {"push": {"branches": [trigger_branch]}}

    dump_for_gha(azure_template, outfile)


def build_linux_pipeline(
    stages,
    trigger_branch,
    script=azure_unix_script,
    azure_template=None,
    runs_on="ubuntu-latest",
    outfile="linux.yml",
    pipeline_name="build_linux",
    setup_pixi_version: str = DEFAULT_SETUP_PIXI_VERSION,
    pixi_version: str = DEFAULT_PIXI_VERSION,
):
    build_unix_pipeline(
        stages,
        trigger_branch,
        script=script,
        azure_template=azure_template,
        runs_on=runs_on,
        outfile=outfile,
        pipeline_name=pipeline_name,
        target="linux-64",
        setup_pixi_version=setup_pixi_version,
        pixi_version=pixi_version,
    )


def build_osx_pipeline(
    stages,
    trigger_branch,
    vm_imagename="macos-15-intel",
    outfile="osx.yml",
    azure_template=None,
    script=azure_unix_script,
    target="osx-64",
    pipeline_name="build_osx64",
    setup_pixi_version: str = DEFAULT_SETUP_PIXI_VERSION,
    pixi_version: str = DEFAULT_PIXI_VERSION,
):
    build_unix_pipeline(
        stages,
        trigger_branch,
        script=script,
        azure_template=azure_template,
        runs_on=vm_imagename,
        outfile=outfile,
        target=target,
        pipeline_name=pipeline_name,
        setup_pixi_version=setup_pixi_version,
        pixi_version=pixi_version,
    )


def build_win_pipeline(
    stages,
    trigger_branch,
    outfile="win.yml",
    azure_template=None,
    setup_pixi_version: str = DEFAULT_SETUP_PIXI_VERSION,
    pixi_version: str = DEFAULT_PIXI_VERSION,
):
    vm_imagename = "windows-2022"
    # Build Win pipeline
    blurb = {"jobs": {}, "name": "build_win"}

    if azure_template is None:
        azure_template = blurb

    script = azure_win_script

    # overwrite with what we're finding in the repo!
    if os.path.exists(".scripts/build_win.bat"):
        with open(".scripts/build_win.bat", "r") as fi:
            script = lu(fi.read())

    prev_batch_keys = []
    for i, s in enumerate(stages):
        stage_name = f"stage_{i}"
        batch_keys = []
        for batch in s:
            batch_key = f"{stage_name}_job_{len(azure_template['jobs'])}"
            batch_keys.append(batch_key)

            pretty_stage_name = get_stage_name(batch)

            build_env = {
                "ANACONDA_API_TOKEN": "${{ secrets.ANACONDA_API_TOKEN }}",
                "CURRENT_RECIPES": f"{' '.join([pkg for pkg in batch])}",
                "PYTHONUNBUFFERED": 1,
            }

            steps = [
                {"name": "Checkout code", "uses": "actions/checkout@v7"},
                get_setup_pixi_step(setup_pixi_version, pixi_version),
                {
                    "uses": "egor-tensin/cleanup-path@v5",
                    "with": {
                        "dirs": "C:\\Program Files\\Git\\usr\\bin;C:\\Program Files\\Git\\bin;C:\\Program Files\\Git\\cmd;C:\\Program Files\\Git\\mingw64\\bin"
                    },
                },
                {
                    "shell": "cmd",
                    "run": azure_win_preconfig_script,
                    "name": "conda-forge build setup",
                },
                {
                    "shell": "cmd",
                    "run": script,
                    "env": build_env,
                    "name": f"Build {' '.join([pkg for pkg in batch])}",
                },
            ]

            job = {
                "name": pretty_stage_name,
                "runs-on": vm_imagename,
                "strategy": {"fail-fast": False},
                "needs": prev_batch_keys,
                "env": {
                    "CONDA_BLD_PATH": "C:\\\\bld\\\\",
                    "VINCA_CUSTOM_CMAKE_BUILD_DIR": "C:\\\\x\\\\",
                },
                "steps": steps,
            }

            job["permissions"] = {
                "id-token": "write",
                "attestations": "write",
            }

            azure_template["jobs"][batch_key] = job

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
    distro = Distro(
        temp_vinca_conf["ros_distro"],
        python_version,
        temp_vinca_conf["_snapshot"],
        temp_vinca_conf["_additional_packages_snapshot"],
    )

    all_packages = get_selected_packages(distro, temp_vinca_conf)
    temp_vinca_conf["_selected_pkgs"] = all_packages

    all_outputs = generate_outputs(distro, temp_vinca_conf)
    return all_outputs


def main():
    args = parse_command_line(sys.argv)

    full_tree = get_full_tree()
    setup_pixi_version = config.setup_pixi_version or DEFAULT_SETUP_PIXI_VERSION
    pixi_version = config.pixi_version or DEFAULT_PIXI_VERSION

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

    platform = args.platform

    if len(metas) >= 1:
        requirements = {}

        for pkg in full_tree + additional_recipes:
            if "outputs" in pkg:
                req_section = pkg["outputs"][0]["requirements"]
            else:
                req_section = pkg["requirements"]
            requirements[pkg["package"]["name"]] = req_section.get(
                "host", []
            ) + req_section.get("run", [])

        # Normalize direct and conditional requirements to package names.
        for pkg_name, reqs in requirements.items():
            requirements[pkg_name] = extract_dependency_names(reqs)

        test_requirements = add_test_requirements(requirements, metas)

        G = build_requirement_graph(requirements, test_requirements)

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
        build_unix_pipeline(
            stages,
            args.trigger_branch,
            outfile="linux.yml",
            pipeline_name="build_linux64",
            setup_pixi_version=setup_pixi_version,
            pixi_version=pixi_version,
        )

    if args.platform == "osx-64":
        build_osx_pipeline(
            stages,
            args.trigger_branch,
            setup_pixi_version=setup_pixi_version,
            pixi_version=pixi_version,
        )

    if args.platform == "osx-arm64":
        build_osx_pipeline(
            stages,
            args.trigger_branch,
            vm_imagename="macos-15",
            outfile="osx_arm64.yml",
            script=azure_unix_script,
            target=platform,
            pipeline_name="build_osx_arm64",
            setup_pixi_version=setup_pixi_version,
            pixi_version=pixi_version,
        )

    if args.platform == "linux-aarch64":
        # Build aarch64 pipeline
        build_unix_pipeline(
            stages,
            args.trigger_branch,
            runs_on="ubuntu-24.04-arm",
            outfile="linux_aarch64.yml",
            target=platform,
            pipeline_name="build_linux_aarch64",
            setup_pixi_version=setup_pixi_version,
            pixi_version=pixi_version,
        )

    # windows
    if args.platform == "win-64":
        build_win_pipeline(
            stages,
            args.trigger_branch,
            outfile="win.yml",
            setup_pixi_version=setup_pixi_version,
            pixi_version=pixi_version,
        )

    if args.platform == "emscripten-wasm32":
        build_unix_pipeline(
            stages,
            args.trigger_branch,
            outfile="emscripten_wasm32.yml",
            pipeline_name="build_emscripten_wasm32",
            target="emscripten-wasm32",
            setup_pixi_version=setup_pixi_version,
            pixi_version=pixi_version,
        )
