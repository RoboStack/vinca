from vinca import generate_gha
from vinca.pipeline import BuildTimings, batch_stages, get_all_ancestors


def test_github_generator_loads_packaged_build_scripts():
    assert "build_unix.sh" in generate_gha.unix_build_script


def test_get_all_ancestors_follows_transitive_dependencies_and_cycles():
    graph = {
        "ros-app": ["ros-library", "python"],
        "ros-library": ["ros-core"],
        "ros-core": ["ros-app"],
    }

    assert get_all_ancestors(graph, "ros-app") == {"ros-library", "ros-core"}


def test_get_all_ancestors_reaches_the_bottom_of_a_deep_graph():
    # Dependencies more than one level away have to be included, otherwise the CI
    # stages schedule a package before the things it transitively needs.
    graph = {
        "ros-app": ["ros-nav", "boost"],
        "ros-nav": ["ros-tf", "eigen"],
        "ros-tf": ["ros-geometry-msgs"],
        "ros-geometry-msgs": ["ros-std-msgs"],
        "ros-std-msgs": ["ros2-rosidl"],
        "ros2-rosidl": [],
        "ros-unrelated": ["ros-app"],
    }

    assert get_all_ancestors(graph, "ros-app") == {
        "ros-nav",
        "ros-tf",
        "ros-geometry-msgs",
        "ros-std-msgs",
        "ros2-rosidl",
    }


def test_get_all_ancestors_stops_at_non_ros_dependencies():
    graph = {
        "ros-app": ["boost"],
        "boost": ["ros-should-not-be-reached"],
    }

    assert get_all_ancestors(graph, "ros-app") == set()


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


def test_batch_stages_accepts_the_legacy_own_stage_key(tmp_path):
    config = tmp_path / "vinca.yaml"
    config.write_text("build_in_own_azure_stage: [ros-special]\n")

    assert batch_stages([["ros-a", "ros-special"]], 3, config_path=config) == [
        [["ros-special"]],
        [["ros-a"]],
    ]


def test_batch_stages_splits_large_stages(tmp_path):
    config = tmp_path / "vinca.yaml"
    config.write_text("{}\n")

    assert batch_stages([["a", "b", "c", "d", "e"]], 2, config_path=config) == [
        [["a", "b"], ["c", "d"], ["e"]]
    ]


def test_batch_stages_packs_by_time(tmp_path):
    config = tmp_path / "vinca.yaml"
    config.write_text("{}\n")
    timings = BuildTimings(
        recipes={"heavy": 600, "big": 300, "mid": 200},
        default=10,
        job_overhead=0,
        runners=4,
    )
    stage = ["a", "heavy", "b", "big", "c", "mid", "d", "e"]

    # in order and by count, heavy and big would share a job; by time, heavy gets its
    # own job and the rest fill the others (each job keeps the stage order)
    batches = batch_stages([stage], 25, config_path=config, timings=timings)

    assert len(batches) == 1
    jobs = batches[0]
    assert ["heavy"] in jobs
    assert sorted(p for job in jobs for p in job) == sorted(stage)
    longest = max(sum(timings.seconds(p) for p in job) for job in jobs)
    assert longest == 600
    for job in jobs:
        assert job == [p for p in stage if p in job]


def test_batch_stages_with_timings_respects_the_batch_size(tmp_path):
    config = tmp_path / "vinca.yaml"
    config.write_text("{}\n")
    timings = BuildTimings(recipes={}, default=1, job_overhead=30, runners=2)

    # cheap packages: as few jobs as the batch size allows (the overhead dominates)
    batches = batch_stages(
        [[f"p{i}" for i in range(10)]], 4, config_path=config, timings=timings
    )

    assert sorted(map(len, batches[0])) == [3, 3, 4]


def test_batch_stages_with_timings_shares_jobs_between_short_stages(tmp_path):
    config = tmp_path / "vinca.yaml"
    config.write_text("{}\n")
    timings = BuildTimings(
        recipes={"long": 1000}, default=10, job_overhead=30, runners=8
    )

    batches = batch_stages(
        [["a"], ["b", "c"], ["long"], ["d"]], 25, config_path=config, timings=timings
    )

    # a, b and c are short, so they share one job; long has its own, then d
    assert batches == [[["a", "b", "c"]], [["long"]], [["d"]]]


def test_build_timings_from_file(tmp_path):
    path = tmp_path / "timings.json"
    path.write_text('{"recipes": {"a": 5, "b": 7, "c": 100}, "job_overhead": 12}')

    timings = BuildTimings.from_file(path, runners=3)

    assert timings.seconds("c") == 100
    assert timings.seconds("unknown") == 7  # the median
    assert (timings.job_overhead, timings.runners) == (12, 3)
