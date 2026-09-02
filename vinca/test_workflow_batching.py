"""Tests for configurable workflow batching strategies."""

import networkx as nx
import pytest

from vinca.generate_gha import build_unix_pipeline, build_win_pipeline
from vinca.workflow_batching import (
    WorkflowBatchingConfig,
    available_batching_strategies,
    estimate_recipe_weights,
    parse_workflow_batching_config,
    plan_workflow_batches,
)


@pytest.mark.parametrize("vinca_config", [{}, None])
def test_legacy_strategy_is_the_default(vinca_config):
    batching_config = parse_workflow_batching_config(vinca_config)

    assert batching_config.strategy == "legacy"
    assert available_batching_strategies() == ("legacy", "schedule-and-isolation")


def test_batching_config_rejects_a_non_mapping_root():
    with pytest.raises(ValueError, match="vinca.yaml must contain a mapping"):
        parse_workflow_batching_config([])


def test_legacy_strategy_preserves_stage_barriers():
    graph = nx.DiGraph()
    plan = plan_workflow_batches(
        [["one", "two"], ["three"]],
        graph,
        {},
        [],
        2,
        WorkflowBatchingConfig(),
    )

    assert plan.batches == (("one", "two"), ("three",))
    assert plan.dependencies == ((), (0,))


def test_recipe_weights_account_for_build_backend_and_overrides():
    recipes = [
        {"package": {"name": "compat"}},
        {"package": {"name": "explicit-empty"}, "build": {"script": ""}},
        {
            "package": {"name": "python-package"},
            "build": {"script": "$RECIPE_DIR/build_ament_python.sh"},
        },
        {
            "package": {"name": "cpp-package"},
            "build": {"script": "$RECIPE_DIR/build_ament_cmake.sh"},
        },
        {
            "package": {"name": "cpp-vendor"},
            "build": {"script": "$RECIPE_DIR/build_ament_cmake.sh"},
            "source": {"patches": ["one.patch"]},
        },
    ]

    weights = estimate_recipe_weights(
        recipes,
        build_backend_weights={"ament_python": 1.5},
        package_weights={"cpp-package": 12.0},
    )

    assert weights["compat"] == weights["explicit-empty"]
    assert weights["compat"] < weights["python-package"] < weights["cpp-vendor"]
    assert weights["cpp-package"] == 12.0


def test_batching_config_accepts_weight_calibration():
    batching_config = parse_workflow_batching_config(
        {
            "github_actions": {
                "batching": {
                    "strategy": "schedule-and-isolation",
                    "max_jobs": 10,
                    "build_backend_weights": {"ament_python": 2.5},
                    "package_weights": {"ros2-rclcpp": 17.0},
                }
            }
        }
    )

    assert batching_config.build_backend_weights["ament_python"] == 2.5
    assert batching_config.package_weights == {"ros2-rclcpp": 17.0}


def test_schedule_strategy_requires_a_job_limit():
    with pytest.raises(ValueError, match="max_jobs is required"):
        parse_workflow_batching_config(
            {"github_actions": {"batching": {"strategy": "schedule-and-isolation"}}}
        )


def test_unknown_strategy_reports_available_choices():
    with pytest.raises(
        ValueError, match="choose one of: legacy, schedule-and-isolation"
    ):
        parse_workflow_batching_config(
            {"github_actions": {"batching": {"strategy": "unknown"}}}
        )


def test_schedule_strategy_caps_jobs_and_preserves_dependency_order():
    graph = nx.DiGraph(
        [
            ("foundation", "left"),
            ("foundation", "right"),
            ("left", "join"),
            ("right", "join"),
            ("join", "consumer"),
            ("independent", "consumer"),
        ]
    )
    batching_config = WorkflowBatchingConfig(
        strategy="schedule-and-isolation",
        max_jobs=3,
        runner_count=2,
    )

    first = plan_workflow_batches(
        [["foundation", "independent"], ["left", "right"], ["join"], ["consumer"]],
        graph,
        {package: 1.0 for package in graph},
        [],
        5,
        batching_config,
    )
    second = plan_workflow_batches(
        [["foundation", "independent"], ["left", "right"], ["join"], ["consumer"]],
        graph,
        {package: 1.0 for package in graph},
        [],
        5,
        batching_config,
    )

    assert first == second
    assert len(first.batches) == 3
    assert sorted(package for batch in first.batches for package in batch) == sorted(
        graph.nodes
    )

    package_to_batch = {
        package: batch_index
        for batch_index, batch in enumerate(first.batches)
        for package in batch
    }
    batch_graph = nx.DiGraph()
    batch_graph.add_nodes_from(range(len(first.batches)))
    for batch_index, dependencies in enumerate(first.dependencies):
        assert all(dependency < batch_index for dependency in dependencies)
        batch_graph.add_edges_from(
            (dependency, batch_index) for dependency in dependencies
        )
    assert nx.is_directed_acyclic_graph(batch_graph)

    for dependency, consumer in graph.edges:
        dependency_batch = package_to_batch[dependency]
        consumer_batch = package_to_batch[consumer]
        if dependency_batch != consumer_batch:
            assert nx.has_path(batch_graph, dependency_batch, consumer_batch)

    for batch in first.batches:
        positions = {package: index for index, package in enumerate(batch)}
        for dependency, consumer in graph.subgraph(batch).edges:
            assert positions[dependency] < positions[consumer]


def test_schedule_strategy_rejects_stage_graph_mismatches():
    batching_config = WorkflowBatchingConfig(
        strategy="schedule-and-isolation",
        max_jobs=1,
    )

    with pytest.raises(ValueError, match="different packages"):
        plan_workflow_batches(
            [["one"]],
            nx.DiGraph(),
            {"one": 1.0},
            [],
            5,
            batching_config,
        )


def test_schedule_strategy_keeps_configured_packages_isolated():
    graph = nx.DiGraph([("foundation", "consumer"), ("side", "consumer")])
    batching_config = WorkflowBatchingConfig(
        strategy="schedule-and-isolation",
        max_jobs=2,
        runner_count=2,
    )

    plan = plan_workflow_batches(
        [["foundation", "side"], ["consumer"]],
        graph,
        {package: 1.0 for package in graph},
        ["foundation"],
        5,
        batching_config,
    )

    assert ("foundation",) in plan.batches


def test_schedule_strategy_can_batch_disconnected_packages():
    graph = nx.DiGraph()
    graph.add_nodes_from(("one", "two", "three"))
    batching_config = WorkflowBatchingConfig(
        strategy="schedule-and-isolation",
        max_jobs=1,
        runner_count=8,
    )

    plan = plan_workflow_batches(
        [["one", "two", "three"]],
        graph,
        {package: 1.0 for package in graph},
        [],
        5,
        batching_config,
    )

    assert plan.batches == (("one", "three", "two"),)
    assert plan.dependencies == ((),)


@pytest.mark.parametrize("workflow_builder", [build_unix_pipeline, build_win_pipeline])
def test_schedule_plan_renders_exact_unix_and_windows_needs(
    workflow_builder, tmp_path, monkeypatch
):
    monkeypatch.chdir(tmp_path)
    yaml = pytest.importorskip("yaml")
    graph = nx.DiGraph(
        [
            ("foundation", "python-package"),
            ("foundation", "cpp-package"),
            ("python-package", "consumer"),
            ("cpp-package", "consumer"),
            ("independent", "consumer"),
        ]
    )
    recipes = [
        {
            "package": {"name": "python-package"},
            "build": {"script": "$RECIPE_DIR/build_ament_python.sh"},
        },
        {
            "package": {"name": "cpp-package"},
            "build": {"script": "$RECIPE_DIR/build_ament_cmake.sh"},
        },
    ]
    batching_config = parse_workflow_batching_config(
        {
            "github_actions": {
                "batching": {
                    "strategy": "schedule-and-isolation",
                    "max_jobs": 4,
                    "runner_count": 2,
                    "maximum_batch_size": 3,
                    "build_backend_weights": {"ament_cmake": 9.0},
                }
            }
        }
    )
    weights = {package: 1.0 for package in graph}
    weights.update(
        estimate_recipe_weights(
            recipes,
            batching_config.build_backend_weights,
            batching_config.package_weights,
        )
    )
    plan = plan_workflow_batches(
        [
            ["foundation", "independent"],
            ["python-package", "cpp-package"],
            ["consumer"],
        ],
        graph,
        weights,
        ["foundation"],
        5,
        batching_config,
    )

    assert len(plan.batches) <= batching_config.max_jobs
    assert ("foundation",) in plan.batches
    assert weights["cpp-package"] > weights["python-package"]

    outfile = tmp_path / "workflow.yml"
    workflow_builder(
        plan.as_stages(),
        "buildbranch",
        outfile=outfile,
        batch_dependencies=plan.dependencies,
    )
    workflow = yaml.safe_load(outfile.read_text())
    job_keys = [f"stage_{index}_job_{index}" for index in range(len(plan.batches))]
    for batch_index, dependencies in enumerate(plan.dependencies):
        assert workflow["jobs"][job_keys[batch_index]]["needs"] == [
            job_keys[dependency] for dependency in dependencies
        ]

    batch_graph = nx.DiGraph()
    batch_graph.add_nodes_from(range(len(plan.batches)))
    batch_graph.add_edges_from(
        (dependency, batch_index)
        for batch_index, dependencies in enumerate(plan.dependencies)
        for dependency in dependencies
    )
    package_to_batch = {
        package: batch_index
        for batch_index, batch in enumerate(plan.batches)
        for package in batch
    }
    assert nx.is_directed_acyclic_graph(batch_graph)
    for dependency, consumer in graph.edges:
        if package_to_batch[dependency] != package_to_batch[consumer]:
            assert nx.has_path(
                batch_graph, package_to_batch[dependency], package_to_batch[consumer]
            )


def test_schedule_strategy_reports_an_impossible_batch_size_limit():
    graph = nx.DiGraph()
    graph.add_nodes_from(("one", "two", "three"))
    batching_config = WorkflowBatchingConfig(
        strategy="schedule-and-isolation",
        max_jobs=1,
        maximum_batch_size=2,
    )

    with pytest.raises(ValueError, match="maximum_batch_size"):
        plan_workflow_batches(
            [["one", "two", "three"]],
            graph,
            {package: 1.0 for package in graph},
            [],
            5,
            batching_config,
        )
