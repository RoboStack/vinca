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

import shutil
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any, Optional

from rattler_build import (
    JinjaConfig,
    PlatformConfig,
    RattlerBuildError,
    RenderConfig,
    Stage0Recipe,
    VariantConfig,
)

from vinca.utils import built_for_distro

Build = tuple[str, str, str]


def published_builds(records: Iterable[dict[str, Any]], ros_distro: str) -> set[Build]:
    """(name, version, build string) of every record built for *ros_distro*."""
    return {
        (record["name"], record["version"], record["build"])
        for record in records
        if built_for_distro(record, ros_distro)
    }


def render_outputs(
    recipes_dir: Path, target_platform: str, variant_config: Optional[Path]
) -> dict[str, list[Build]]:
    """Render every recipe in *recipes_dir*, grouped by package name (the name of
    the recipe's directory, see template.write_recipe).

    Rendered as built natively (build platform = target platform), as the packages
    are: the build platform is part of the variant hash, so rendering on another
    machine (e.g. one linux-64 runner generating every platform's recipes) gives
    build strings that never match the published ones."""
    platform = PlatformConfig(
        target_platform=target_platform,
        build_platform=target_platform,
        host_platform=target_platform,
    )
    variants = (
        VariantConfig.from_files_with_context(
            [variant_config], JinjaConfig(platform=platform)
        )
        if variant_config is not None
        else VariantConfig()
    )
    render_config = RenderConfig(platform=platform)
    rendered: dict[str, list[Build]] = {}
    for recipe_path in sorted(recipes_dir.glob("**/recipe.yaml")):
        try:
            variants_of_recipe = Stage0Recipe.from_file(recipe_path).render(
                variants, render_config
            )
        except RattlerBuildError as error:
            raise RuntimeError(f"rendering {recipe_path} failed: {error}") from error
        for variant in variants_of_recipe:
            recipe = variant.recipe
            name = str(recipe.package.name)
            build = (name, str(recipe.package.version), str(recipe.build.string))
            rendered.setdefault(name, []).append(build)
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
