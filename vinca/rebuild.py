"""Plan which packages a change of the conda_build_config.yaml pins has to rebuild.

    vinca-rebuild-plan --old old/conda_build_config.yaml --new conda_build_config.yaml

A package is rebuilt when

* it uses a changed pin: one of its build or host requirements is a key of
  conda_build_config.yaml whose value changed (the variant it is built with changes);
* or it depends, directly or transitively, on a package that is rebuilt (host or run
  requirement): ROS packages compile against each other's headers, so an ABI change
  propagates even to packages that don't depend on the changed library themselves.

Everything else keeps its published build, so instead of a full rebuild only these
packages get a new build number (and the mutex, when its run_constraints change).
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict, deque
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

import ruamel.yaml

from vinca.pinning import (
    DEFAULT_PLATFORMS,
    _platform_configurations,
    _walk_requirements,
)

# conda_build_config.yaml keys that are not variants of a dependency
_NOT_PINS = {"zip_keys", "pin_run_as_build", "extend_keys", "ignore_version"}

ChangedPins = dict[str, tuple[Optional[Any], Optional[Any]]]


def normalized(name: str) -> str:
    return name.strip().lower().replace("_", "-")


def _load(path: Path) -> dict[str, Any]:
    data = ruamel.yaml.YAML(typ="safe").load(Path(path).read_text(encoding="utf-8"))
    return data or {}


def changed_pins(old: Mapping[str, Any], new: Mapping[str, Any]) -> ChangedPins:
    """Pins whose values differ, keyed by normalized dependency name.

    A changed ``pin_run_as_build`` entry (how tightly the run dependency is pinned)
    counts as a change of that pin too.
    """
    changed: ChangedPins = {}
    for key in set(old) | set(new):
        if key.startswith("__") or key in _NOT_PINS:
            continue
        before, after = old.get(key), new.get(key)
        if before != after:
            changed[normalized(key)] = (before, after)
    old_run = old.get("pin_run_as_build") or {}
    new_run = new.get("pin_run_as_build") or {}
    for key in set(old_run) | set(new_run):
        if old_run.get(key) != new_run.get(key) and normalized(key) not in changed:
            changed[normalized(key)] = (old.get(key), new.get(key))
    return changed


def _names(requirements: Any) -> set[str]:
    names = set()
    for requirement in _walk_requirements(requirements):
        if isinstance(requirement, str) and not requirement.startswith("${{"):
            names.add(normalized(requirement.split()[0]))
    return names


def plan(
    requirements: Mapping[str, Mapping[str, Any]],
    changed: Iterable[str],
    package_prefix: str,
) -> dict[str, str]:
    """Packages to rebuild, mapped to the reason.

    ``requirements`` maps each ROS package name to its ``build``/``host``/``run``
    requirements (as vinca generates them); ``package_prefix`` is the conda name
    prefix of the ROS packages (e.g. ``ros2``).
    """
    changed = {normalized(name) for name in changed}
    prefix = normalized(package_prefix) + "-"
    by_conda_suffix = {normalized(name): name for name in requirements}

    reasons: dict[str, str] = {}
    dependents: dict[str, set[str]] = defaultdict(set)
    for name, reqs in requirements.items():
        used = sorted((_names(reqs.get("build")) | _names(reqs.get("host"))) & changed)
        if used:
            reasons[name] = "uses " + ", ".join(used)
        for dependency in _names(reqs.get("host")) | _names(reqs.get("run")):
            if dependency.startswith(prefix):
                ros_name = by_conda_suffix.get(dependency[len(prefix) :])
                if ros_name and ros_name != name:
                    dependents[ros_name].add(name)

    queue = deque(sorted(reasons))
    while queue:
        name = queue.popleft()
        for dependent in sorted(dependents[name]):
            if dependent not in reasons:
                reasons[dependent] = f"depends on {name}"
                queue.append(dependent)
    return reasons


def requirements_from_vinca(
    base_dir: str | Path, platforms: Sequence[str] = DEFAULT_PLATFORMS
) -> tuple[dict[str, dict[str, list[Any]]], str]:
    """Requirements of every selected package, combined over the platforms, and the
    conda name prefix of the ROS packages."""
    from vinca.main import generate_output
    from vinca.naming import get_package_prefix

    combined: dict[str, dict[str, list[Any]]] = {}
    prefix = ""
    with _platform_configurations(base_dir, platforms) as configurations:
        for _, distro, vinca_config, group_packages in configurations:
            prefix = get_package_prefix(distro, vinca_config)
            for name in vinca_config["_selected_pkgs"]:
                if not distro.check_package(name):
                    continue
                reqs = generate_output(
                    name,
                    vinca_config,
                    distro,
                    distro.get_version(name),
                    group_packages,
                    dependencies_only=True,
                )
                if reqs is None:
                    continue
                target = combined.setdefault(name, {"build": [], "host": [], "run": []})
                for group in target:
                    target[group].append(reqs.get(group))
    return combined, prefix


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("\n", 2)[2],
    )
    parser.add_argument(
        "--old", required=True, type=Path, help="previous conda_build_config.yaml"
    )
    parser.add_argument(
        "--new", required=True, type=Path, help="new conda_build_config.yaml"
    )
    parser.add_argument(
        "--platform",
        action="append",
        help=f"platforms whose packages are considered (default: {', '.join(DEFAULT_PLATFORMS)})",
    )
    parser.add_argument(
        "--vinca-dir", type=Path, default=Path("."), help="directory of vinca.yaml"
    )
    parser.add_argument("--json", type=Path, help="write the plan to this file")
    args = parser.parse_args(argv)

    changed = changed_pins(_load(args.old), _load(args.new))
    result: dict[str, Any] = {
        "changed_pins": {k: [v[0], v[1]] for k, v in sorted(changed.items())}
    }
    if changed:
        requirements, prefix = requirements_from_vinca(
            args.vinca_dir, args.platform or DEFAULT_PLATFORMS
        )
        rebuild = plan(requirements, changed, prefix)
        result.update(packages=len(requirements), rebuild=dict(sorted(rebuild.items())))
    else:
        result.update(packages=None, rebuild={})

    print("Changed pins: " + (", ".join(sorted(changed)) or "none"))
    rebuild = result["rebuild"]
    if changed:
        direct = sum(1 for reason in rebuild.values() if reason.startswith("uses "))
        print(
            f"Rebuild {len(rebuild)} of {result['packages']} packages ({direct} use a changed pin directly):"
        )
        for name, reason in rebuild.items():
            print(f"  {name}: {reason}")
    if args.json:
        args.json.write_text(
            json.dumps(result, indent=2, default=str) + "\n", encoding="utf-8"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
