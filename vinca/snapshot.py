import argparse
import yaml
from .distro import Distro


def main():
    parser = argparse.ArgumentParser(
        description="Dependency snapshotting tool for ROS packages"
    )
    parser.add_argument(
        "-d",
        "--distro",
        type=str,
        dest="distro",
        default="humble",
        help="ROS distribution to use (default: humble)",
        required=False,
    )
    parser.add_argument(
        "-p",
        "--package",
        type=str,
        dest="package",
        default="ros_base",
        help="ROS package to get dependencies for (default: ros_base)",
        required=False,
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        dest="output",
        default="snapshot.yaml",
        help="Output file to write dependencies to",
        required=False,
    )
    parser.add_argument(
        "-q",
        "--quiet",
        dest="quiet",
        action="store_true",
        help="Suppress output to stdout",
        required=False,
    )
    args = parser.parse_args()

    distro = Distro(args.distro)
    deps = distro.get_depends(args.package)
    deps.add(args.package)

    if not args.quiet:
        max_len = max([len(dep) for dep in deps])
        print("\033[1m{0:{2}} {1}\033[0m".format("Package", "Version", max_len + 2))

    output = {}

    for dep in deps:
        url, tag = distro.get_released_repo(dep)
        version = distro.get_version(dep)
        output[dep] = {"url": url, "version": version, "tag": tag}

        if not args.quiet:
            print("{0:{2}} {1}".format(dep, version, max_len + 2))

    with open(args.output, "w") as f:
        yaml.dump(output, f)
