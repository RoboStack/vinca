from collections import Counter

import pytest
import ruamel.yaml

from .workflow_timings import (
    BuildTiming,
    estimate_job_overhead,
    extract_build_timing,
    fit_package_weights,
    write_timing_config,
)


def _job(conclusion="success", build_conclusion="success"):
    return {
        "name": "python-package cpp-package",
        "conclusion": conclusion,
        "started_at": "2026-01-01T00:00:00Z",
        "completed_at": "2026-01-01T00:12:00Z",
        "steps": [
            {
                "name": "Set up job",
                "conclusion": "success",
                "started_at": "2026-01-01T00:00:00Z",
                "completed_at": "2026-01-01T00:01:00Z",
            },
            {
                "name": "Build python-package cpp-package",
                "conclusion": build_conclusion,
                "started_at": "2026-01-01T00:01:00Z",
                "completed_at": "2026-01-01T00:11:00Z",
            },
        ],
    }


def test_extract_build_timing_uses_successful_build_step():
    timing = extract_build_timing(_job(), 42)

    assert timing == BuildTiming(
        packages=("python-package", "cpp-package"),
        build_seconds=600,
        overhead_seconds=120,
        run_id=42,
        job_name="python-package cpp-package",
    )


@pytest.mark.parametrize(
    ("job_conclusion", "build_conclusion"),
    [("failure", "success"), ("success", "failure")],
)
def test_extract_build_timing_rejects_incomplete_builds(
    job_conclusion, build_conclusion
):
    assert extract_build_timing(_job(job_conclusion, build_conclusion), 42) is None


def test_estimate_job_overhead_uses_time_outside_build_step():
    timings = [
        BuildTiming(("one",), 240, 60, 1, "one"),
        BuildTiming(("two",), 420, 60, 1, "two"),
        BuildTiming(("one", "two"), 600, 60, 1, "one two"),
    ]

    assert estimate_job_overhead(timings) == pytest.approx(1.0)


def test_fit_package_weights_uses_timings_and_backend_priors():
    timings = [
        BuildTiming(("python",), 60, 10, 1, "python"),
        BuildTiming(("cpp",), 600, 10, 1, "cpp"),
        BuildTiming(("python", "cpp"), 660, 10, 2, "python cpp"),
    ]

    weights, sample_counts = fit_package_weights(
        timings,
        {"python": 1.0, "cpp": 4.0},
        regularization=0,
        job_overhead=10 / 60,
    )

    assert weights == pytest.approx({"python": 1.0, "cpp": 10.0})
    assert sample_counts == {"cpp": 2, "python": 2}


def test_write_timing_config_emits_batching_configuration(tmp_path):
    timing = BuildTiming(("python",), 60, 30, 7, "python")
    output = tmp_path / "weights.yaml"

    write_timing_config(
        output,
        [timing],
        {"python": 1.0},
        {"python": 1},
        "RoboStack/ros-jazzy",
        "linux.yml",
        8,
        120,
        Counter(success=1),
        Counter(success=1),
        0.5,
    )

    yaml = ruamel.yaml.YAML(typ="safe")
    document = yaml.load(output)
    batching = document["github_actions"]["batching"]
    assert batching == {
        "strategy": "schedule-and-isolation",
        "max_jobs": 120,
        "runner_count": 8,
        "job_overhead": 0.5,
        "package_weights": {"python": 1.0},
    }
    assert document["workflow_timing_metadata"]["runs"] == 1
    assert document["workflow_timing_metadata"]["run_conclusions"] == {"success": 1}
