import argparse
from ruamel.yaml import YAML
from .utils import get_repodata, get_pinnings


def main():
    parser = argparse.ArgumentParser(
        description="Dependency migration tool for ROS packages"
    )
    parser.add_argument(
        type=str,
        dest="pinnings",
        help="Path to the local pinnings file",
        metavar="PINNINGS",
    )
    parser.add_argument(
        "-a",
        "--all",
        action="store_true",
        dest="all",
        help="Show all dependencies",
        required=False,
    )
    parser.add_argument(
        "-p",
        "--platform",
        type=str,
        dest="platform",
        choices=["win-64", "linux-64", "linux-aarch64", "osx-64", "osx-arm64"],
        default="linux-64",
        help="Platform to target (default: linux-64)",
        required=False,
    )
    parser.add_argument(
        "--repodata",
        type=str,
        dest="repodata",
        default="https://conda.anaconda.org/robostack-staging/",
        help="URL to the repodata file",
        required=False,
    )
    parser.add_argument(
        "--upstream",
        type=str,
        dest="upstream",
        default="https://raw.githubusercontent.com/conda-forge/conda-forge-pinning-feedstock/refs/heads/main/recipe/conda_build_config.yaml",
        help="URL to the upstream pinnings file",
        required=False,
    )
    args = parser.parse_args()

    repodata = get_repodata(args.repodata, args.platform)
    packages = repodata["packages"]

    deps = set()
    for pkg in packages:
        for dep in packages[pkg].get("depends", []):
            deps.add(dep.split()[0])

    local = get_pinnings(args.pinnings)
    upstream = get_pinnings(args.upstream)

    common = sorted(deps.intersection(local.keys()))
    max_len = max(len(name) for name in common)
    print("\033[1m{0:{2}} {1}\033[0m".format("Package", "Versions", max_len))

    changed = []

    for name in common:
        current = local[name]
        latest = upstream[name]

        if current == latest:
            if args.all:
                print("{0:{2}} {1}".format(name, current, max_len))
            continue

        print("{0:{3}} {1} -> {2}".format(name, current, latest, max_len))

        local[name] = latest
        changed.append(name)

    if not changed:
        print("No packages to migrate")
        return

    with open(args.pinnings, "w") as f:
        yaml = YAML()
        yaml.indent(mapping=2, sequence=4, offset=2)
        yaml.compact_seq_seq = False
        # TODO: check output formatting
        yaml.dump(local, f)
