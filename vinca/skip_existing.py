"""Skip recipes whose exact builds are already published.

A package is only already built when a build with the same name, version and build
string is on a channel: the build string carries the variant hash (e.g. ``py312`` /
``py314``) and the build number, so a new python version or a bumped build number is
built even though the package name already exists. vinca renders the generated
recipes with rattler-build to get those build strings and removes the recipes whose
every output is published; rattler-build (``--skip-existing=all``) then skips the
published variants of the recipes that remain.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any, Optional

from vinca.utils import built_for_distro

Build = tuple[str, str, str]


def published_builds(
    repodatas: Iterable[Mapping[str, Any]], ros_distro: str
) -> set[Build]:
    """(name, version, build string) of every record built for *ros_distro*."""
    builds = set()
    for repodata in repodatas:
        records = {**repodata.get("packages", {}), **repodata.get("packages.conda", {})}
        for record in records.values():
            if built_for_distro(record, ros_distro):
                builds.add((record["name"], record["version"], record["build"]))
    return builds


def render_outputs(
    recipes_dir: Path, target_platform: str, variant_config: Optional[Path]
) -> dict[str, list[Build]]:
    """Render every recipe in *recipes_dir*, grouped by package name (the name of
    the recipe's directory, see template.write_recipe)."""
    cmd = [
        "rattler-build",
        "build",
        "--render-only",
        "--recipe-dir",
        str(recipes_dir),
        "--target-platform",
        target_platform,
    ]
    if variant_config is not None:
        cmd += ["-m", str(variant_config)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"rendering the recipes in {recipes_dir} failed:\n{proc.stderr}"
        )
    rendered: dict[str, list[Build]] = {}
    for output in json.loads(proc.stdout):
        package = output["recipe"]["package"]
        build = (
            package["name"],
            package["version"],
            output["recipe"]["build"]["string"],
        )
        rendered.setdefault(package["name"], []).append(build)
    return rendered


def built_recipes(
    rendered: Mapping[str, list[Build]], published: set[Build]
) -> set[str]:
    """Recipes whose every rendered output is published."""
    return {
        name
        for name, builds in rendered.items()
        if builds and all(build in published for build in builds)
    }


def remove_built_recipes(
    recipes_dir: Path,
    target_platform: str,
    variant_config: Optional[Path],
    published: set[Build],
) -> set[str]:
    """Delete the recipe directories that have nothing left to build; return their names."""
    rendered = render_outputs(recipes_dir, target_platform, variant_config)
    removed = built_recipes(rendered, published)
    for name in removed:
        shutil.rmtree(recipes_dir / name)
    return removed
