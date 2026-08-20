"""Shared helpers for CI pipeline generation."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

import yaml

from vinca.utils import get_repodata


def _chunks(items: list[str], size: int) -> list[list[str]]:
    return [items[index : index + size] for index in range(0, len(items), size)]


def batch_stages(
    stages: Iterable[Iterable[str]],
    max_batch_size: int = 5,
    *,
    config_path: str | Path = "vinca.yaml",
) -> list[list[list[str]]]:
    """Group dependency stages into CI batches without mutating the input.

    The outer list preserves sequential dependency stages. Each inner list contains
    batches which can execute in parallel.
    """
    if max_batch_size < 1:
        raise ValueError("max_batch_size must be at least 1")

    with Path(config_path).open(encoding="utf-8") as stream:
        vinca_conf = yaml.safe_load(stream) or {}
    build_individually = set(vinca_conf.get("build_in_own_stage", []))

    result: list[list[list[str]]] = []
    pending: list[str] = []
    for original_stage in stages:
        stage = list(original_stage)
        own_stage = [package for package in stage if package in build_individually]
        stage = [package for package in stage if package not in build_individually]

        if own_stage:
            if pending:
                result.append([pending])
                pending = []
            for package in own_stage:
                result.append([[package]])

        if len(stage) >= max_batch_size:
            if pending:
                result.append([pending])
                pending = []
            result.append(_chunks(stage, max_batch_size))
        elif len(pending) + len(stage) < max_batch_size:
            pending.extend(stage)
        else:
            if pending:
                result.append([pending])
            pending = stage

    if pending:
        result.append([pending])
    return result


def get_skip_existing(vinca_conf: Mapping[str, Any], platform: str) -> list[dict]:
    """Fetch repodata files configured for skip-existing checks."""
    repositories = vinca_conf.get("skip_existing") or []
    repodatas = []
    for repository in repositories:
        print(f"Fetching repodata: {repository}")
        repodatas.append(get_repodata(repository, platform))
    return repodatas


def get_all_ancestors(graph: Mapping[str, Iterable[str]], node: str) -> set[str]:
    """Return all transitive ROS dependencies of *node*.

    Missing nodes and dependency cycles are handled safely.
    """
    ancestors: set[str] = set()
    visited = {node}
    pending = list(graph.get(node, ()))
    while pending:
        dependency = pending.pop()
        if dependency in visited:
            continue
        visited.add(dependency)
        if not (dependency.startswith("ros-") or dependency.startswith("ros2-")):
            continue
        ancestors.add(dependency)
        pending.extend(graph.get(dependency, ()))
    return ancestors
