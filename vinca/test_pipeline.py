from vinca import generate_gha
from vinca.pipeline import batch_stages, get_all_ancestors


def test_github_generator_loads_packaged_build_scripts():
    assert "build_unix.sh" in generate_gha.unix_build_script


def test_get_all_ancestors_follows_transitive_dependencies_and_cycles():
    graph = {
        "ros-app": ["ros-library", "python"],
        "ros-library": ["ros-core"],
        "ros-core": ["ros-app"],
    }

    assert get_all_ancestors(graph, "ros-app") == {"ros-library", "ros-core"}


def test_batch_stages_does_not_mutate_input(tmp_path):
    config = tmp_path / "vinca.yaml"
    config.write_text("build_in_own_stage: [ros-special]\n")
    stages = [["ros-a", "ros-special"], ["ros-b", "ros-c"], ["ros-d", "ros-e"]]
    original = [stage.copy() for stage in stages]

    batches = batch_stages(stages, 3, config_path=config)

    assert stages == original
    assert batches == [
        [["ros-special"]],
        [["ros-a"]],
        [["ros-b", "ros-c"]],
        [["ros-d", "ros-e"]],
    ]


def test_batch_stages_splits_large_stages(tmp_path):
    config = tmp_path / "vinca.yaml"
    config.write_text("{}\n")

    assert batch_stages([["a", "b", "c", "d", "e"]], 2, config_path=config) == [
        [["a", "b"], ["c", "d"], ["e"]]
    ]
