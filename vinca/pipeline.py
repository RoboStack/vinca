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

# a stage this short is one job, shared with the short stages next to it
SHARED_JOB_SECONDS = 120.0


def _chunks(items: list[str], size: int) -> list[list[str]]:
    return [items[index : index + size] for index in range(0, len(items), size)]


def default_runners(platform: str) -> int:
    """GitHub's default limit on concurrent jobs with its hosted runners (the Free plan):
    20 in total, of which 5 macOS."""
    return 5 if platform.startswith("osx") else 20


@dataclass(frozen=True)
class BuildTimings:
    """Measured build times, for packing each stage's packages into jobs by time.

    ``recipes`` maps package names to their build (and upload) seconds; packages
    without a measurement count as ``default`` seconds. ``job_overhead`` is the fixed
    cost of a CI job (checkout, environment setup, dispatch). ``runners`` is the number
    of jobs that can run at once (default: GitHub's limit, see default_runners).
    """

    recipes: Mapping[str, float]
    default: float = 20.0
    job_overhead: float = 30.0
    runners: int = 20

    @classmethod
    def from_file(
        cls, path: str | Path, runners: int | None = None, platform: str = ""
    ) -> BuildTimings:
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
            runners=runners or int(data.get("runners", default_runners(platform))),
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


def _span(jobs: list[list[str]], timings: BuildTimings) -> float:
    """How long a stage of these jobs takes on the runners."""
    return _makespan(
        [timings.job_overhead + sum(map(timings.seconds, job)) for job in jobs],
        timings.runners,
    )


def batch_stages(
    stages: Iterable[Iterable[str]],
    max_batch_size: int = 5,
    *,
    config_path: str | Path = "vinca.yaml",
    timings: BuildTimings | None = None,
    followers: Mapping[str, list[str]] | None = None,
    job_budget: int | None = None,
) -> list[list[list[str]]]:
    """Group dependency stages into CI batches without mutating the input.

    The outer list preserves sequential dependency stages. Each inner list contains
    batches which can execute in parallel.

    Each stage first gets as few jobs as max_batch_size (packages per job) allows, and
    consecutive short stages (one job of at most SHARED_JOB_SECONDS each) share a job.
    Then, as long as the job budget lasts, the stage whose duration one more job cuts the
    most gets it, its packages packed into its jobs longest first: the jobs go where they
    shorten the workflow most, and a smaller budget (e.g. for GitHub's workflow size
    limit) still leaves the slowest stages split. A stage gets at most one job per
    runner. Without timings every package counts the same (BuildTimings' default).

    followers maps a package to packages that are built right after it in its job (e.g.
    a compatibility package that only depends on it): they leave their own stage, and
    their time counts towards their leader's.
    """
    if max_batch_size < 1:
        raise ValueError("max_batch_size must be at least 1")
    timings = timings or BuildTimings(recipes={})

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

    # units in order: (packages, splittable); a unit that isn't splittable is one job
    units: list[tuple[list[str], bool]] = []
    shared: list[str] = []  # consecutive short stages sharing one job

    def flush() -> None:
        nonlocal shared
        if shared:
            units.append((shared, False))
            shared = []

    for stage in stages:
        own = [p for p in stage if p in build_individually]
        stage = [p for p in stage if p not in build_individually]
        if own:
            flush()
            units.extend(([p], False) for p in own)
        if not stage:
            continue
        short = sum(map(timings.seconds, stage)) <= SHARED_JOB_SECONDS
        if short and len(stage) <= max_batch_size:
            if len(shared) + len(stage) > max_batch_size:
                flush()
            shared = shared + stage
            continue
        flush()
        units.append((stage, True))
    flush()

    # as few jobs as the batch size allows, then more where they cut the most time
    jobs: list[list[list[str]]] = []
    for packages, splittable in units:
        if not splittable:
            jobs.append([packages])
            continue
        count = -(-len(packages) // max_batch_size)
        packed = _pack(packages, count, timings)
        jobs.append(
            packed
            if all(len(job) <= max_batch_size for job in packed)
            else _chunks(packages, max_batch_size)
        )

    def more(index: int) -> tuple[float, list[list[str]]] | None:
        """The unit's next packing with more jobs that is faster (e.g. 8 equal packages
        only get faster again at 8 jobs once there are 4), and the time it saves per
        added job."""
        packages, splittable = units[index]
        current = jobs[index]
        most = min(len(packages), timings.runners)
        if not splittable:
            return None
        span = _span(current, timings)
        for count in range(len(current) + 1, most + 1):
            packed = _pack(packages, count, timings)
            if not all(len(job) <= max_batch_size for job in packed):
                continue
            saved = span - _span(packed, timings)
            # not for a negligible gain: every job makes the workflow larger
            if saved > 0.01 * span:
                return saved / (len(packed) - len(current)), packed
        return None

    total = sum(len(stage) for stage in jobs)
    candidates = []
    for index in range(len(units)):
        if (option := more(index)) is not None:
            heapq.heappush(candidates, (-option[0], index))
    while candidates and (job_budget is None or total < job_budget):
        _, index = heapq.heappop(candidates)
        option = more(index)  # recomputed: the unit's jobs may have changed
        if option is None:
            continue
        if (
            job_budget is not None
            and total + len(option[1]) - len(jobs[index]) > job_budget
        ):
            continue
        total += len(option[1]) - len(jobs[index])
        jobs[index] = option[1]
        if (option := more(index)) is not None:
            heapq.heappush(candidates, (-option[0], index))

    if following:
        jobs = [
            [[q for p in job for q in [p, *follow.get(p, [])]] for job in stage]
            for stage in jobs
        ]
    return jobs


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
