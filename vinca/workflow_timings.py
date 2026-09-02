"""Extract empirical package build weights from GitHub Actions history."""

from __future__ import annotations

import argparse
import os
import statistics
import subprocess
import time
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Mapping, Optional, Sequence
from urllib.parse import quote

import requests
import ruamel.yaml

from vinca.workflow_batching import estimate_recipe_weights


@dataclass(frozen=True)
class BuildTiming:
    """Represent one successful workflow build step and its measured durations."""

    packages: tuple[str, ...]
    build_seconds: float
    overhead_seconds: float
    run_id: int
    job_name: str


class GitHubActionsClient:
    """Read paginated workflow run and job metadata from the GitHub REST API."""

    def __init__(self, repository: str, token: Optional[str] = None):
        if repository.count("/") != 1:
            raise ValueError("repository must use the OWNER/NAME form")
        self.repository = repository
        self.session = requests.Session()
        self.session.headers.update(
            {
                "Accept": "application/vnd.github+json",
                "X-GitHub-Api-Version": "2022-11-28",
                "User-Agent": "vinca-workflow-timings",
            }
        )
        if token:
            self.session.headers["Authorization"] = f"Bearer {token}"

    def workflow_runs(
        self, workflow: str, limit: int, branch: Optional[str] = None
    ) -> list[Mapping[str, object]]:
        """Return the newest completed runs for one workflow file or ID."""
        params = {"status": "completed"}
        if branch:
            params["branch"] = branch
        endpoint = f"/repos/{self.repository}/actions/workflows/{quote(workflow, safe='')}/runs"
        return self._paginated(endpoint, "workflow_runs", limit, params)

    def run_jobs(self, run_id: int) -> list[Mapping[str, object]]:
        """Return every job from the latest attempt of a workflow run."""
        endpoint = f"/repos/{self.repository}/actions/runs/{run_id}/jobs"
        return self._paginated(endpoint, "jobs", None, {"filter": "latest"})

    def _paginated(
        self,
        endpoint: str,
        collection_key: str,
        limit: Optional[int],
        params: Mapping[str, object],
    ) -> list[Mapping[str, object]]:
        results = []
        page = 1
        while limit is None or len(results) < limit:
            request_params = dict(params)
            request_params.update({"page": page, "per_page": 100})
            response = self._get(endpoint, request_params)
            payload = response.json()
            page_results = payload.get(collection_key, [])
            if not isinstance(page_results, list):
                raise RuntimeError(
                    f"GitHub returned an invalid {collection_key!r} collection"
                )
            results.extend(page_results)
            if len(page_results) < 100:
                break
            page += 1
        return results if limit is None else results[:limit]

    def _get(self, endpoint: str, params: Mapping[str, object]) -> requests.Response:
        response = None
        for attempt in range(5):
            response = self.session.get(
                f"https://api.github.com{endpoint}",
                params=params,
                timeout=60,
            )
            if response.status_code not in {429, 500, 502, 503, 504}:
                response.raise_for_status()
                return response
            time.sleep(2**attempt)
        if response is None:
            raise RuntimeError("GitHub request was not attempted")
        response.raise_for_status()
        return response


def extract_build_timing(
    job: Mapping[str, object], run_id: int
) -> Optional[BuildTiming]:
    """Extract a successful ``Build …`` step from one GitHub Actions job."""
    if job.get("conclusion") != "success":
        return None
    steps = job.get("steps")
    if not isinstance(steps, Sequence) or isinstance(steps, (str, bytes)):
        return None
    build_step = next(
        (
            step
            for step in steps
            if isinstance(step, Mapping)
            and isinstance(step.get("name"), str)
            and step["name"].startswith("Build ")
        ),
        None,
    )
    if build_step is None or build_step.get("conclusion") != "success":
        return None
    packages = tuple(build_step["name"].removeprefix("Build ").split())
    if not packages:
        return None
    build_seconds = _duration_seconds(build_step)
    job_seconds = _duration_seconds(job)
    if build_seconds is None or job_seconds is None or build_seconds <= 0:
        return None
    return BuildTiming(
        packages=packages,
        build_seconds=build_seconds,
        overhead_seconds=max(0.0, job_seconds - build_seconds),
        run_id=run_id,
        job_name=str(job.get("name", "")),
    )


def collect_build_timings(
    client: GitHubActionsClient,
    workflow: str,
    run_limit: int,
    branch: Optional[str] = None,
) -> tuple[list[BuildTiming], Counter, Counter]:
    """Collect successful build observations and run and job conclusion counts."""
    timings = []
    run_conclusions = Counter()
    job_conclusions = Counter()
    for run in client.workflow_runs(workflow, run_limit, branch):
        run_conclusions[str(run.get("conclusion"))] += 1
        run_id = run.get("id")
        if not isinstance(run_id, int):
            continue
        for job in client.run_jobs(run_id):
            job_conclusions[str(job.get("conclusion"))] += 1
            timing = extract_build_timing(job, run_id)
            if timing is not None:
                timings.append(timing)
    return timings, run_conclusions, job_conclusions


def estimate_job_overhead(timings: Sequence[BuildTiming]) -> float:
    """Estimate fixed job minutes spent outside the generated build step."""
    if not timings:
        raise ValueError("No successful build timings were found")
    return statistics.median(timing.overhead_seconds / 60.0 for timing in timings)


def fit_package_weights(
    timings: Sequence[BuildTiming],
    prior_weights: Mapping[str, float],
    regularization: float = 3.0,
    iterations: int = 60,
    job_overhead: float = 0.0,
) -> tuple[dict[str, float], dict[str, int]]:
    """Fit positive additive package minutes with ridge-regularized coordinate descent."""
    if not timings:
        raise ValueError("No successful build timings were found")
    if regularization < 0:
        raise ValueError("regularization must be non-negative")

    grouped_durations = defaultdict(list)
    for timing in timings:
        grouped_durations[tuple(sorted(set(timing.packages)))].append(
            max(
                0.01,
                (timing.build_seconds + timing.overhead_seconds) / 60.0 - job_overhead,
            )
        )
    observations = [
        (packages, statistics.median(durations), len(durations))
        for packages, durations in sorted(grouped_durations.items())
    ]
    observed_packages = sorted(
        {package for packages, _, _ in observations for package in packages}
    )
    scale_candidates = []
    for packages, duration, sample_count in observations:
        prior_sum = sum(
            max(0.01, prior_weights.get(package, 1.0)) for package in packages
        )
        scale_candidates.extend([duration / prior_sum] * sample_count)
    prior_scale = statistics.median(scale_candidates)
    scaled_priors = {
        package: max(0.01, prior_weights.get(package, 1.0) * prior_scale)
        for package in observed_packages
    }
    weights = dict(scaled_priors)

    observations_by_package = defaultdict(list)
    for observation_index, (packages, _, _) in enumerate(observations):
        for package in packages:
            observations_by_package[package].append(observation_index)

    for _ in range(iterations):
        for package in observed_packages:
            numerator = regularization * scaled_priors[package]
            denominator = regularization
            for observation_index in observations_by_package[package]:
                packages, duration, sample_count = observations[observation_index]
                other_weight = sum(
                    weights[other] for other in packages if other != package
                )
                numerator += sample_count * (duration - other_weight)
                denominator += sample_count
            weights[package] = max(0.01, numerator / max(denominator, 1e-9))

    sample_counts = {
        package: sum(
            observations[index][2] for index in observations_by_package[package]
        )
        for package in observed_packages
    }
    return weights, sample_counts


def load_recipe_priors(recipes_directory: Path) -> dict[str, float]:
    """Load recipe files and derive backend-aware prior package weights."""
    yaml = ruamel.yaml.YAML(typ="safe")
    recipes = []
    for path in sorted(recipes_directory.glob("**/recipe.yaml")):
        with path.open("r", encoding="utf-8") as stream:
            recipe = yaml.load(stream)
        if isinstance(recipe, Mapping):
            recipes.append(recipe)
    return estimate_recipe_weights(recipes)


def write_timing_config(
    output_path: Path,
    timings: Sequence[BuildTiming],
    package_weights: Mapping[str, float],
    sample_counts: Mapping[str, int],
    repository: str,
    workflow: str,
    runner_count: int,
    max_jobs: int,
    run_conclusions: Mapping[str, int],
    job_conclusions: Mapping[str, int],
    job_overhead: float,
) -> None:
    """Write a config-compatible YAML snippet and bounded provenance metadata."""
    document = {
        "github_actions": {
            "batching": {
                "strategy": "schedule-and-isolation",
                "max_jobs": max_jobs,
                "runner_count": runner_count,
                "job_overhead": round(job_overhead, 4),
                "package_weights": {
                    package: round(weight, 4)
                    for package, weight in sorted(package_weights.items())
                },
            }
        },
        "workflow_timing_metadata": {
            "repository": repository,
            "workflow": workflow,
            "observations": len(timings),
            "runs": len({timing.run_id for timing in timings}),
            "run_ids": sorted({timing.run_id for timing in timings}),
            "run_conclusions": dict(sorted(run_conclusions.items())),
            "job_conclusions": dict(sorted(job_conclusions.items())),
            "package_samples": dict(sorted(sample_counts.items())),
        },
    }
    yaml = ruamel.yaml.YAML()
    yaml.indent(mapping=2, sequence=4, offset=2)
    with output_path.open("w", encoding="utf-8") as stream:
        yaml.dump(document, stream)


def _duration_seconds(item: Mapping[str, object]) -> Optional[float]:
    started_at = item.get("started_at")
    completed_at = item.get("completed_at")
    if not isinstance(started_at, str) or not isinstance(completed_at, str):
        return None
    start = datetime.fromisoformat(started_at.replace("Z", "+00:00"))
    end = datetime.fromisoformat(completed_at.replace("Z", "+00:00"))
    return (end - start).total_seconds()


def _github_token() -> Optional[str]:
    token = os.environ.get("GH_TOKEN") or os.environ.get("GITHUB_TOKEN")
    if token:
        return token
    try:
        result = subprocess.run(
            ["gh", "auth", "token"],
            check=True,
            capture_output=True,
            text=True,
            timeout=15,
        )
    except (
        FileNotFoundError,
        subprocess.CalledProcessError,
        subprocess.TimeoutExpired,
    ):
        return None
    return result.stdout.strip() or None


def parse_command_line(arguments: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parse command-line options for historical workflow timing extraction."""
    parser = argparse.ArgumentParser(
        description="Fit Vinca package weights from GitHub Actions job timings"
    )
    parser.add_argument("--repository", required=True, help="GitHub OWNER/NAME")
    parser.add_argument(
        "--workflow",
        default="linux.yml",
        help="Workflow file name or numeric workflow ID",
    )
    parser.add_argument("--branch", help="Optional workflow run branch")
    parser.add_argument(
        "--runs", type=int, default=30, help="Completed runs to inspect"
    )
    parser.add_argument(
        "--recipes", type=Path, default=Path("recipes"), help="Generated recipes"
    )
    parser.add_argument("--output", type=Path, default=Path("workflow-weights.yaml"))
    parser.add_argument("--runner-count", type=int, default=8)
    parser.add_argument("--max-jobs", type=int, default=120)
    parser.add_argument("--regularization", type=float, default=3.0)
    parsed = parser.parse_args(arguments)
    for name in ("runs", "runner_count", "max_jobs"):
        if getattr(parsed, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if parsed.regularization < 0:
        parser.error("--regularization must be non-negative")
    return parsed


def main(arguments: Optional[Sequence[str]] = None) -> None:
    """Extract workflow timings and write package weights for Vinca batching."""
    parsed = parse_command_line(arguments)
    client = GitHubActionsClient(parsed.repository, _github_token())
    timings, run_conclusions, job_conclusions = collect_build_timings(
        client, parsed.workflow, parsed.runs, parsed.branch
    )
    priors = load_recipe_priors(parsed.recipes)
    job_overhead = estimate_job_overhead(timings)
    package_weights, sample_counts = fit_package_weights(
        timings,
        priors,
        parsed.regularization,
        job_overhead=job_overhead,
    )
    if priors:
        package_weights = {
            package: weight
            for package, weight in package_weights.items()
            if package in priors
        }
        sample_counts = {
            package: count
            for package, count in sample_counts.items()
            if package in priors
        }
    write_timing_config(
        parsed.output,
        timings,
        package_weights,
        sample_counts,
        parsed.repository,
        parsed.workflow,
        parsed.runner_count,
        parsed.max_jobs,
        run_conclusions,
        job_conclusions,
        job_overhead,
    )
    print(
        f"Wrote {len(package_weights)} package weights from {len(timings)} "
        f"successful jobs to {parsed.output}"
    )


if __name__ == "__main__":
    main()
