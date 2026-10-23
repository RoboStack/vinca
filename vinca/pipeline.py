"""Shared helpers for CI pipeline generation.

The CI generators turn a dependency graph over conda package names into an ordered
sequence of build stages. This module owns the graph and batching logic that is
independent of any particular CI provider; the provider-specific job and workflow
rendering lives in :mod:`vinca.generate_gha` and :mod:`vinca.generate_gitlab`.
"""

from __future__ import annotations

import heapq
import json
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import yaml

from vinca.utils import get_repodata

# with timings: consecutive single-job stages share a job while it stays this short
SHARED_JOB_SECONDS = 120.0


def _chunks(items: list[str], size: int) -> list[list[str]]:
    return [items[index : index + size] for index in range(0, len(items), size)]


@dataclass(frozen=True)
class BuildTimings:
    """Measured build times, for packing each stage's packages into jobs by time.

    ``recipes`` maps package names to their build (and upload) seconds; packages
    without a measurement count as ``default`` seconds. ``job_overhead`` is the fixed
    cost of a CI job (checkout, environment setup, dispatch). ``runners`` is the number
    of jobs that can run at once.
    """

    recipes: Mapping[str, float]
    default: float = 20.0
    job_overhead: float = 30.0
    runners: int = 8

    @classmethod
    def from_file(cls, path: str | Path, runners: int | None = None) -> BuildTimings:
        """Read a JSON file ``{"recipes": {name: seconds}, "default": s, "job_overhead": s}``."""
        with Path(path).open(encoding="utf-8") as stream:
            data = json.load(stream)
        recipes = {
            name: float(seconds)
            for name, seconds in (data.get("recipes") or {}).items()
        }
        known = sorted(recipes.values())
        default = data.get("default", known[len(known) // 2] if known else cls.default)
        return cls(
            recipes=recipes,
            default=float(default),
            job_overhead=float(data.get("job_overhead", cls.job_overhead)),
            runners=runners or int(data.get("runners", cls.runners)),
        )

    def seconds(self, package: str) -> float:
        return self.recipes.get(package, self.default)


def _makespan(durations: list[float], runners: int) -> float:
    """When the last of these jobs ends, started longest first on this many runners."""
    free = [0.0] * min(runners, len(durations))
    heapq.heapify(free)
    for duration in sorted(durations, reverse=True):
        heapq.heappush(free, heapq.heappop(free) + duration)
    return max(free, default=0.0)


def _pack(stage: list[str], jobs: int, timings: BuildTimings) -> list[list[str]]:
    """Longest-processing-time packing of the stage's packages into this many jobs, each
    job's packages kept in stage order."""
    order = sorted(range(len(stage)), key=lambda i: (-timings.seconds(stage[i]), i))
    loads = [(0.0, job) for job in range(jobs)]
    heapq.heapify(loads)
    members: list[list[int]] = [[] for _ in range(jobs)]
    for index in order:
        load, job = heapq.heappop(loads)
        members[job].append(index)
        heapq.heappush(loads, (load + timings.seconds(stage[index]), job))
    return [[stage[i] for i in sorted(job)] for job in members if job]


def _pack_by_time(
    stage: list[str], max_batch_size: int, timings: BuildTimings
) -> list[list[str]]:
    """The stage's packages in as few jobs as reach the shortest stage (within 1%),
    given the runners and the overhead of each job; no job gets more than
    max_batch_size packages."""
    fewest = -(-len(stage) // max_batch_size)
    most = max(fewest, min(len(stage), 4 * timings.runners))
    candidates = []
    for jobs in range(fewest, most + 1):
        packed = _pack(stage, jobs, timings)
        if any(len(job) > max_batch_size for job in packed):
            continue
        durations = [
            timings.job_overhead + sum(map(timings.seconds, job)) for job in packed
        ]
        candidates.append((_makespan(durations, timings.runners), packed))
    if not candidates:
        return _chunks(stage, max_batch_size)
    # fewer jobs (a smaller workflow) when that costs at most 1%
    shortest = min(span for span, _ in candidates)
    return next(packed for span, packed in candidates if span <= shortest * 1.01)


def batch_stages(
    stages: Iterable[Iterable[str]],
    max_batch_size: int = 5,
    *,
    config_path: str | Path = "vinca.yaml",
    timings: BuildTimings | None = None,
    followers: Mapping[str, list[str]] | None = None,
) -> list[list[list[str]]]:
    """Group dependency stages into CI batches without mutating the input.

    The outer list preserves sequential dependency stages. Each inner list contains
    batches which can execute in parallel.

    Without timings, a stage is cut into jobs of max_batch_size packages in order, and
    consecutive small stages share a job. With timings, each stage is packed into jobs
    by measured build time (longest first), so that no single job holds up the stage;
    consecutive stages that need just one job share it while it stays short.

    followers maps a package to packages that are built right after it in its job (e.g.
    a compatibility package that only depends on it): they leave their own stage, and
    with timings their time counts towards their leader's.
    """
    if max_batch_size < 1:
        raise ValueError("max_batch_size must be at least 1")

    stages = [list(stage) for stage in stages]
    present = {package for stage in stages for package in stage}
    follow = {
        leader: [f for f in members if f in present]
        for leader, members in (followers or {}).items()
        if leader in present
    }
    following = {f for members in follow.values() for f in members}
    if following:
        stages = [[p for p in stage if p not in following] for stage in stages]
        stages = [stage for stage in stages if stage]
        if timings is not None:
            recipes = dict(timings.recipes)
            for leader, members in follow.items():
                recipes[leader] = timings.seconds(leader) + sum(
                    map(timings.seconds, members)
                )
            timings = replace(timings, recipes=recipes)

    with Path(config_path).open(encoding="utf-8") as stream:
        vinca_conf = yaml.safe_load(stream) or {}
    build_individually = set(
        vinca_conf.get("build_in_own_stage")
        # Accepted for compatibility with configurations written for the Azure generator.
        or vinca_conf.get("build_in_own_azure_stage")
        or []
    )

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

        if timings is not None:
            if not stage:
                continue
            seconds = sum(map(timings.seconds, stage))
            if seconds <= SHARED_JOB_SECONDS and len(stage) <= max_batch_size:
                # a short stage: one job, shared with the stages before it while that stays short
                if (
                    pending
                    and len(pending) + len(stage) <= max_batch_size
                    and sum(map(timings.seconds, pending)) + seconds
                    <= SHARED_JOB_SECONDS
                ):
                    pending.extend(stage)
                    continue
                if pending:
                    result.append([pending])
                pending = stage
                continue
            if pending:
                result.append([pending])
                pending = []
            result.append(_pack_by_time(stage, max_batch_size, timings))
            continue

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
    if following:
        result = [
            [[q for p in job for q in [p, *follow.get(p, [])]] for job in stage]
            for stage in result
        ]
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
    if not graph.get(node):
        print(f"[yellow]{node} not found")
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
