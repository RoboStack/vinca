import networkx as nx
import yaml
import re
import glob
import sys, os
import textwrap
import argparse
from distutils.dir_util import copy_tree
import yaml
import requests

parsed_args = None


def parse_command_line(argv):
    parser = argparse.ArgumentParser(
        description="Conda recipe Dot graphic generator for ROS packages"
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
