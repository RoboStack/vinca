"""Configurable package batching strategies for generated workflows."""

from __future__ import annotations

import heapq
import json
import math
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Callable, Mapping, Optional, Sequence

import networkx as nx


DEFAULT_BUILD_BACKEND_WEIGHTS = {
    "empty": 0.1,
    "ament_python": 1.0,
    "other": 2.0,
    "cmake": 3.0,
    "catkin": 4.0,
    "ament_cmake": 4.0,
}


@dataclass(frozen=True)
class WorkflowBatchingConfig:
    """Configure how generated recipes are grouped into workflow jobs."""

    strategy: str = "legacy"
    max_jobs: Optional[int] = None
    runner_count: int = 8
    job_overhead: float = 5.0
    maximum_batch_size: Optional[int] = None
    schedule_tolerance: float = 0.01
    failure_isolation_weight: float = 1.0
    build_backend_weights: Mapping[str, float] = field(
        default_factory=lambda: dict(DEFAULT_BUILD_BACKEND_WEIGHTS)
    )
    package_weights: Mapping[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class BatchPlan:
    """Describe ordered package batches and their direct batch dependencies."""

    batches: tuple[tuple[str, ...], ...]
    dependencies: tuple[tuple[int, ...], ...]

    def as_stages(self) -> list[list[list[str]]]:
        """Return one compatibility stage per batch for pipeline rendering."""
        return [[list(batch)] for batch in self.batches]


@dataclass(frozen=True)
class _StrategyInput:
    stages: tuple[tuple[str, ...], ...]
    dependency_graph: nx.DiGraph
    weights: Mapping[str, float]
    isolated_packages: frozenset[str]
    legacy_batch_size: int
    config: WorkflowBatchingConfig


BatchingStrategy = Callable[[_StrategyInput], BatchPlan]
_BATCHING_STRATEGIES: dict[str, BatchingStrategy] = {}


def register_batching_strategy(
    name: str,
) -> Callable[[BatchingStrategy], BatchingStrategy]:
    """Register a workflow batching strategy under a configuration name."""

    def register(strategy: BatchingStrategy) -> BatchingStrategy:
        if name in _BATCHING_STRATEGIES:
            raise ValueError(
                f"The workflow batching strategy {name!r} is already registered"
            )
        _BATCHING_STRATEGIES[name] = strategy
        return strategy

    return register


def available_batching_strategies() -> tuple[str, ...]:
    """Return the deterministic list of configured workflow batching strategies."""
    return tuple(sorted(_BATCHING_STRATEGIES))


def parse_workflow_batching_config(
    vinca_config: Optional[Mapping[str, object]],
) -> WorkflowBatchingConfig:
    """Parse and validate the GitHub Actions batching section of ``vinca.yaml``."""
    if vinca_config is None:
        vinca_config = {}
    if not isinstance(vinca_config, Mapping):
        raise ValueError("vinca.yaml must contain a mapping")
    github_actions = vinca_config.get("github_actions", {})
    if github_actions is None:
        github_actions = {}
    if not isinstance(github_actions, Mapping):
        raise ValueError("github_actions must be a mapping")

    batching = github_actions.get("batching", {})
    if batching is None:
        batching = {}
    if not isinstance(batching, Mapping):
        raise ValueError("github_actions.batching must be a mapping")

    strategy = batching.get("strategy", "legacy")
    if not isinstance(strategy, str) or strategy not in _BATCHING_STRATEGIES:
        choices = ", ".join(available_batching_strategies())
        raise ValueError(
            f"Unknown GitHub Actions batching strategy {strategy!r}; choose one of: {choices}"
        )

    max_jobs = _optional_positive_integer(batching, "max_jobs")
    runner_count = _positive_integer(batching, "runner_count", 8)
    maximum_batch_size = _optional_positive_integer(batching, "maximum_batch_size")
    job_overhead = _non_negative_number(batching, "job_overhead", 5.0)
    schedule_tolerance = _non_negative_number(batching, "schedule_tolerance", 0.01)
    failure_isolation_weight = _non_negative_number(
        batching, "failure_isolation_weight", 1.0
    )
    build_backend_weights = dict(DEFAULT_BUILD_BACKEND_WEIGHTS)
    build_backend_weights.update(_weight_mapping(batching, "build_backend_weights"))
    unknown_backends = set(build_backend_weights) - set(DEFAULT_BUILD_BACKEND_WEIGHTS)
    if unknown_backends:
        names = ", ".join(sorted(unknown_backends))
        raise ValueError(f"Unknown build backend weight names: {names}")
    package_weights = _weight_mapping(batching, "package_weights")

    if strategy != "legacy" and max_jobs is None:
        raise ValueError(
            f"github_actions.batching.max_jobs is required for strategy {strategy!r}"
        )

    return WorkflowBatchingConfig(
        strategy=strategy,
        max_jobs=max_jobs,
        runner_count=runner_count,
        job_overhead=job_overhead,
        maximum_batch_size=maximum_batch_size,
        schedule_tolerance=schedule_tolerance,
        failure_isolation_weight=failure_isolation_weight,
        build_backend_weights=build_backend_weights,
        package_weights=package_weights,
    )


def plan_workflow_batches(
    stages: Sequence[Sequence[str]],
    dependency_graph: nx.DiGraph,
    weights: Mapping[str, float],
    isolated_packages: Sequence[str],
    legacy_batch_size: int,
    batching_config: WorkflowBatchingConfig,
) -> BatchPlan:
    """Create a deterministic batch plan with the selected registered strategy."""
    if batching_config.strategy != "legacy":
        if not nx.is_directed_acyclic_graph(dependency_graph):
            raise ValueError("The package dependency graph must be acyclic")
        stage_packages = [package for stage in stages for package in stage]
        if len(stage_packages) != len(set(stage_packages)):
            raise ValueError("Workflow stages must not contain duplicate packages")
        graph_packages = set(dependency_graph)
        if set(stage_packages) != graph_packages:
            missing = sorted(set(stage_packages) - graph_packages)
            unexpected = sorted(graph_packages - set(stage_packages))
            raise ValueError(
                "Workflow stages and dependency graph contain different packages: "
                f"missing from graph={missing}, missing from stages={unexpected}"
            )

    strategy_input = _StrategyInput(
        stages=tuple(tuple(stage) for stage in stages),
        dependency_graph=dependency_graph.copy(),
        weights=weights,
        isolated_packages=frozenset(isolated_packages),
        legacy_batch_size=legacy_batch_size,
        config=batching_config,
    )
    return _BATCHING_STRATEGIES[batching_config.strategy](strategy_input)


def estimate_recipe_weights(
    recipes: Sequence[Mapping[str, object]],
    build_backend_weights: Optional[Mapping[str, float]] = None,
    package_weights: Optional[Mapping[str, float]] = None,
) -> dict[str, float]:
    """Estimate build costs by backend, recipe complexity, and explicit overrides."""
    backend_weights = dict(DEFAULT_BUILD_BACKEND_WEIGHTS)
    if build_backend_weights is not None:
        backend_weights.update(build_backend_weights)
    explicit_weights = package_weights or {}
    weights = {}
    for recipe in recipes:
        package = recipe.get("package")
        if not isinstance(package, Mapping):
            continue
        name = package.get("name")
        if not isinstance(name, str):
            continue
        if name in explicit_weights:
            weights[name] = float(explicit_weights[name])
            continue

        backend = _recipe_build_backend(recipe)
        requirement_count = _recipe_requirement_count(recipe)
        patch_count = _recipe_patch_count(recipe)
        backend_cost = backend_weights.get(backend, backend_weights["other"])
        vendor_multiplier = 1.5 if "vendor" in name else 1.0
        weights[name] = (
            backend_cost * vendor_multiplier
            + min(requirement_count, 40) * 0.03
            + patch_count * 0.2
        )
    return weights


def _recipe_build_backend(recipe: Mapping[str, object]) -> str:
    build = recipe.get("build")
    if build is None:
        return "empty"
    if not isinstance(build, Mapping):
        return "other"
    script = build.get("script")
    if script is None or script == "" or script == []:
        return "empty"
    script_text = json.dumps(script, sort_keys=True).lower()
    for backend in ("ament_python", "ament_cmake", "catkin", "cmake"):
        if backend in script_text:
            return backend
    return "other"


def _recipe_requirement_count(recipe: Mapping[str, object]) -> int:
    requirement_sections = []
    requirements = recipe.get("requirements")
    if isinstance(requirements, Mapping):
        requirement_sections.append(requirements)
    outputs = recipe.get("outputs")
    if isinstance(outputs, Sequence) and not isinstance(outputs, (str, bytes)):
        for output in outputs:
            if isinstance(output, Mapping) and isinstance(
                output.get("requirements"), Mapping
            ):
                requirement_sections.append(output["requirements"])

    count = 0
    for requirement_section in requirement_sections:
        for section_name in ("host", "run"):
            values = requirement_section.get(section_name, ())
            if isinstance(values, Sequence) and not isinstance(values, (str, bytes)):
                count += len(values)
    return count


def _recipe_patch_count(recipe: Mapping[str, object]) -> int:
    sources = recipe.get("source", ())
    if isinstance(sources, Mapping):
        sources = (sources,)
    if not isinstance(sources, Sequence) or isinstance(sources, (str, bytes)):
        return 0
    count = 0
    for source in sources:
        if not isinstance(source, Mapping):
            continue
        patches = source.get("patches", ())
        if isinstance(patches, Sequence) and not isinstance(patches, (str, bytes)):
            count += len(patches)
    return count


def _positive_integer(config: Mapping[str, object], key: str, default: int) -> int:
    value = config.get(key, default)
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"github_actions.batching.{key} must be a positive integer")
    return value


def _optional_positive_integer(config: Mapping[str, object], key: str) -> Optional[int]:
    value = config.get(key)
    if value is None:
        return None
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"github_actions.batching.{key} must be a positive integer")
    return value


def _weight_mapping(config: Mapping[str, object], key: str) -> dict[str, float]:
    value = config.get(key, {})
    if not isinstance(value, Mapping):
        raise ValueError(f"github_actions.batching.{key} must be a mapping")
    weights = {}
    for name, weight in value.items():
        if not isinstance(name, str):
            raise ValueError(f"github_actions.batching.{key} keys must be strings")
        if (
            not isinstance(weight, (int, float))
            or isinstance(weight, bool)
            or not math.isfinite(weight)
            or weight <= 0
        ):
            raise ValueError(
                f"github_actions.batching.{key}.{name} must be a positive number"
            )
        weights[name] = float(weight)
    return weights


def _non_negative_number(
    config: Mapping[str, object], key: str, default: float
) -> float:
    value = config.get(key, default)
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(value)
        or value < 0
    ):
        raise ValueError(f"github_actions.batching.{key} must be non-negative")
    return float(value)


def _plan_from_stages(stages: Sequence[Sequence[Sequence[str]]]) -> BatchPlan:
    batches = []
    dependencies = []
    previous_stage = []
    for stage in stages:
        current_stage = []
        for batch in stage:
            current_stage.append(len(batches))
            batches.append(tuple(batch))
            dependencies.append(tuple(previous_stage))
        previous_stage = current_stage
    return BatchPlan(tuple(batches), tuple(dependencies))


@register_batching_strategy("legacy")
def _legacy_strategy(strategy_input: _StrategyInput) -> BatchPlan:
    stages = [list(stage) for stage in strategy_input.stages]
    stage_lengths = [len(stage) for stage in stages]
    merged_stages = []
    current_stage = []

    def chunks(values: list[str], size: int):
        for index in range(0, len(values), size):
            yield values[index : index + size]

    for index, stage in enumerate(stages):
        for package in sorted(strategy_input.isolated_packages):
            if package in stage:
                merged_stages.append([[package]])
                stage.remove(package)

        if (
            stage_lengths[index] < strategy_input.legacy_batch_size
            and len(current_stage) + stage_lengths[index]
            < strategy_input.legacy_batch_size
        ):
            current_stage += stage
        else:
            if current_stage:
                merged_stages.append([current_stage])
                current_stage = []
            if stage_lengths[index] < strategy_input.legacy_batch_size:
                current_stage += stage
            else:
                merged_stages.append(
                    list(chunks(stage, strategy_input.legacy_batch_size))
                )
    if current_stage:
        merged_stages.append([current_stage])
    return _plan_from_stages(merged_stages)


def _list_schedule(
    graph: nx.DiGraph, weights: Mapping[int, float], runner_count: int
) -> tuple[dict[int, float], dict[int, float], dict[int, list[int]], float]:
    order = list(nx.lexicographical_topological_sort(graph, key=lambda node: node))
    upward_rank = {}
    for node in reversed(order):
        upward_rank[node] = weights[node] + max(
            (upward_rank[successor] for successor in graph.successors(node)),
            default=0.0,
        )

    indegrees = {node: graph.in_degree(node) for node in graph}
    ready = [(-upward_rank[node], node) for node in graph if indegrees[node] == 0]
    heapq.heapify(ready)
    idle_runners = list(range(runner_count))
    heapq.heapify(idle_runners)
    running = []
    starts = {}
    finishes = {}
    assignments = defaultdict(list)
    current_time = 0.0

    while ready or running:
        while ready and idle_runners:
            _, node = heapq.heappop(ready)
            runner = heapq.heappop(idle_runners)
            starts[node] = current_time
            finishes[node] = current_time + weights[node]
            assignments[runner].append(node)
            heapq.heappush(running, (finishes[node], runner, node))
        if not running:
            raise RuntimeError("The workflow scheduler stalled on an invalid graph")
        current_time = running[0][0]
        completed = []
        while running and running[0][0] <= current_time + 1e-9:
            _, runner, node = heapq.heappop(running)
            heapq.heappush(idle_runners, runner)
            completed.append(node)
        for node in completed:
            for successor in graph.successors(node):
                indegrees[successor] -= 1
                if indegrees[successor] == 0:
                    heapq.heappush(ready, (-upward_rank[successor], successor))

    return starts, finishes, assignments, current_time


def _schedule_metrics(
    graph: nx.DiGraph, weights: Mapping[int, float]
) -> tuple[dict[int, float], dict[int, float]]:
    order = list(nx.lexicographical_topological_sort(graph, key=lambda node: node))
    finish = {}
    for node in order:
        start = max((finish[pred] for pred in graph.predecessors(node)), default=0.0)
        finish[node] = start + weights[node]
    tail = {}
    for node in reversed(order):
        tail[node] = max(
            (
                weights[successor] + tail[successor]
                for successor in graph.successors(node)
            ),
            default=0.0,
        )
    return finish, tail


def _is_safe_contraction(graph: nx.DiGraph, source: int, target: int) -> bool:
    source_reaches_target = nx.has_path(graph, source, target)
    target_reaches_source = nx.has_path(graph, target, source)
    if source_reaches_target and not graph.has_edge(source, target):
        return False
    if target_reaches_source and not graph.has_edge(target, source):
        return False
    if source_reaches_target:
        graph.remove_edge(source, target)
        alternate_path = nx.has_path(graph, source, target)
        graph.add_edge(source, target)
        return not alternate_path
    if target_reaches_source:
        graph.remove_edge(target, source)
        alternate_path = nx.has_path(graph, target, source)
        graph.add_edge(target, source)
        return not alternate_path
    return True


def _merge_graph_nodes(
    graph: nx.DiGraph, source: int, target: int, merged: int
) -> None:
    predecessors = (
        set(graph.predecessors(source)) | set(graph.predecessors(target))
    ) - {
        source,
        target,
    }
    successors = (set(graph.successors(source)) | set(graph.successors(target))) - {
        source,
        target,
    }
    graph.remove_nodes_from((source, target))
    graph.add_node(merged)
    graph.add_edges_from((predecessor, merged) for predecessor in predecessors)
    graph.add_edges_from((merged, successor) for successor in successors)


@register_batching_strategy("schedule-and-isolation")
def _schedule_and_isolation_strategy(strategy_input: _StrategyInput) -> BatchPlan:
    config = strategy_input.config
    if config.max_jobs is None:
        raise ValueError("schedule-and-isolation requires max_jobs")

    package_names = tuple(
        nx.lexicographical_topological_sort(
            strategy_input.dependency_graph, key=lambda package: package
        )
    )
    if len(package_names) <= config.max_jobs:
        batches = tuple((package,) for package in package_names)
        package_to_batch = {
            package: index for index, package in enumerate(package_names)
        }
        return _finalize_plan(
            batches, strategy_input.dependency_graph, package_to_batch
        )

    package_indexes = {package: index for index, package in enumerate(package_names)}
    graph = nx.relabel_nodes(
        strategy_input.dependency_graph, package_indexes, copy=True
    )
    graph = nx.transitive_reduction(graph)
    package_count = len(package_names)
    original_ancestors = {
        package_indexes[package]: {
            package_indexes[ancestor]
            for ancestor in nx.ancestors(strategy_input.dependency_graph, package)
        }
        for package in package_names
    }
    original_descendants = {
        package_indexes[package]: {
            package_indexes[descendant]
            for descendant in nx.descendants(strategy_input.dependency_graph, package)
        }
        for package in package_names
    }
    source_hubs = {
        node
        for node in graph
        if graph.in_degree(node) >= 12
        or graph.out_degree(node) >= 12
        or len(original_descendants[node]) >= package_count // 2
    }
    weights = {
        package_indexes[package]: float(strategy_input.weights.get(package, 1.0))
        for package in package_names
    }
    members = {node: frozenset((node,)) for node in graph}
    ancestor_signatures = {
        node: frozenset(original_ancestors[node] | {node}) for node in graph
    }
    descendant_signatures = {
        node: frozenset(original_descendants[node] | {node}) for node in graph
    }
    contains_hub = {node: node in source_hubs for node in graph}
    contains_isolated = {
        node: package_names[node] in strategy_input.isolated_packages for node in graph
    }
    next_node = package_count

    runner_counts = tuple(
        sorted(
            {
                min(config.max_jobs, max(1, config.runner_count // 2)),
                min(config.max_jobs, config.runner_count),
                min(config.max_jobs, config.runner_count * 2),
            }
        )
    )

    while graph.number_of_nodes() > config.max_jobs:
        reduced = nx.transitive_reduction(graph)
        scheduled_weights = {
            node: weights[node] + config.job_overhead for node in reduced
        }
        critical_finish, tail = _schedule_metrics(reduced, scheduled_weights)
        schedules = {}
        candidate_kinds = {}

        for runner_count in runner_counts:
            starts, finishes, assignments, makespan = _list_schedule(
                reduced, scheduled_weights, runner_count
            )
            schedules[runner_count] = (starts, finishes, makespan)
            for runner_jobs in assignments.values():
                for source, target in zip(runner_jobs, runner_jobs[1:]):
                    pair = tuple(sorted((source, target)))
                    candidate_kinds[pair] = min(candidate_kinds.get(pair, 3), 2)

        for source, target in reduced.edges:
            pair = tuple(sorted((source, target)))
            is_series = (
                reduced.out_degree(source) == 1 and reduced.in_degree(target) == 1
            )
            candidate_kinds[pair] = min(
                candidate_kinds.get(pair, 3), 0 if is_series else 1
            )

        by_sole_successor = defaultdict(list)
        for node in reduced:
            successors = tuple(reduced.successors(node))
            if len(successors) == 1:
                by_sole_successor[successors[0]].append(node)
        for group in by_sole_successor.values():
            ordered = sorted(group, key=lambda node: (weights[node], node))
            for source, target in zip(ordered, ordered[1:]):
                pair = tuple(sorted((source, target)))
                candidate_kinds[pair] = min(candidate_kinds.get(pair, 3), 2)

        candidates = []
        reach_fallback_candidates = []
        for (source, target), kind in candidate_kinds.items():
            if contains_isolated[source] or contains_isolated[target]:
                continue
            merged_size = len(members[source]) + len(members[target])
            if (
                config.maximum_batch_size is not None
                and merged_size > config.maximum_batch_size
            ):
                continue
            if not _is_safe_contraction(reduced, source, target):
                continue

            source_descendants = descendant_signatures[source]
            target_descendants = descendant_signatures[target]
            descendant_similarity = _jaccard_similarity(
                source_descendants, target_descendants
            )
            source_ancestors = ancestor_signatures[source]
            target_ancestors = ancestor_signatures[target]
            ancestor_similarity = _jaccard_similarity(
                source_ancestors, target_ancestors
            )
            comparable = reduced.has_edge(source, target) or reduced.has_edge(
                target, source
            )
            requires_reach_fallback = not comparable and (
                descendant_similarity < 0.5 or ancestor_similarity < 0.5
            )

            external_predecessors = (
                set(reduced.predecessors(source)) | set(reduced.predecessors(target))
            ) - {source, target}
            external_successors = (
                set(reduced.successors(source)) | set(reduced.successors(target))
            ) - {source, target}
            merged_start = max(
                (critical_finish[pred] for pred in external_predecessors), default=0.0
            )
            merged_finish = (
                merged_start + weights[source] + weights[target] + config.job_overhead
            )
            merged_tail = max(
                (
                    scheduled_weights[successor] + tail[successor]
                    for successor in external_successors
                ),
                default=0.0,
            )
            candidate_path = merged_finish + merged_tail
            schedule_penalty = 0.0
            for starts, finishes, makespan in schedules.values():
                overlap = max(
                    0.0,
                    min(finishes[source], finishes[target])
                    - max(starts[source], starts[target]),
                )
                predicted_delay = max(0.0, candidate_path - makespan)
                schedule_penalty = max(
                    schedule_penalty,
                    (predicted_delay + overlap) / max(1.0, makespan),
                )

            reach_distance = 1.0 - (descendant_similarity + ancestor_similarity) / 2.0
            artificial_pairs = sum(
                len(members[target] - original_descendants[package] - {package})
                for package in members[source]
            ) + sum(
                len(members[source] - original_descendants[package] - {package})
                for package in members[target]
            )
            blast_penalty = reach_distance + artificial_pairs / max(1, merged_size)
            spurious_waits = sum(
                not (members[predecessor] & original_ancestors[package])
                for package in members[source] | members[target]
                for predecessor in external_predecessors
            )
            possible_waits = merged_size * len(external_predecessors)
            spurious_wait_severity = (
                spurious_waits / possible_waits if possible_waits else 0.0
            )
            isolation_penalty = config.failure_isolation_weight * (
                blast_penalty + 2.0 * spurious_wait_severity
            )
            schedule_bucket = math.floor(
                schedule_penalty / max(config.schedule_tolerance, 1e-9)
            )
            candidate = (
                0 if kind == 0 else 1,
                0 if kind == 0 else int(contains_hub[source] or contains_hub[target]),
                schedule_bucket,
                isolation_penalty,
                schedule_penalty,
                len(external_predecessors) * len(external_successors),
                weights[source] + weights[target],
                source,
                target,
            )
            if requires_reach_fallback:
                reach_fallback_candidates.append(candidate)
            else:
                candidates.append(candidate)

        if not candidates:
            candidates = reach_fallback_candidates
        candidates.sort()
        merged_this_round = 0
        for candidate in candidates:
            source, target = candidate[-2:]
            if graph.number_of_nodes() <= config.max_jobs:
                break
            if source not in graph or target not in graph:
                continue
            if not _is_safe_contraction(graph, source, target):
                continue
            _merge_graph_nodes(graph, source, target, next_node)
            weights[next_node] = weights[source] + weights[target]
            members[next_node] = members[source] | members[target]
            ancestor_signatures[next_node] = (
                ancestor_signatures[source] | ancestor_signatures[target]
            )
            descendant_signatures[next_node] = (
                descendant_signatures[source] | descendant_signatures[target]
            )
            contains_hub[next_node] = contains_hub[source] or contains_hub[target]
            contains_isolated[next_node] = False
            next_node += 1
            merged_this_round += 1

        if merged_this_round == 0:
            raise ValueError(
                "Cannot satisfy github_actions.batching.max_jobs without exceeding "
                "maximum_batch_size or merging an isolated package"
            )

    batches_by_node = {
        node: tuple(
            nx.lexicographical_topological_sort(
                strategy_input.dependency_graph.subgraph(
                    package_names[index] for index in members[node]
                ),
                key=lambda package: package,
            )
        )
        for node in graph
    }
    quotient_order = tuple(
        nx.lexicographical_topological_sort(
            graph, key=lambda node: batches_by_node[node]
        )
    )
    batches = tuple(batches_by_node[node] for node in quotient_order)
    package_to_batch = {
        package: batch_index
        for batch_index, batch in enumerate(batches)
        for package in batch
    }
    return _finalize_plan(batches, strategy_input.dependency_graph, package_to_batch)


def _jaccard_similarity(left: frozenset[int], right: frozenset[int]) -> float:
    union = left | right
    if not union:
        return 1.0
    return len(left & right) / len(union)


def _finalize_plan(
    batches: tuple[tuple[str, ...], ...],
    dependency_graph: nx.DiGraph,
    package_to_batch: Mapping[str, int],
) -> BatchPlan:
    quotient = nx.DiGraph()
    quotient.add_nodes_from(range(len(batches)))
    quotient.add_edges_from(
        (package_to_batch[source], package_to_batch[target])
        for source, target in dependency_graph.edges
        if package_to_batch[source] != package_to_batch[target]
    )
    if not nx.is_directed_acyclic_graph(quotient):
        raise RuntimeError("The workflow batching strategy produced a cyclic plan")
    reduced = nx.transitive_reduction(quotient)
    dependencies = tuple(
        tuple(sorted(reduced.predecessors(batch_index)))
        for batch_index in range(len(batches))
    )
    return BatchPlan(batches, dependencies)
