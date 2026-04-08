from pathlib import Path

import yaml

from . import generate_azure, generate_gha


def _load_yaml(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_generate_gha_platform_finalize_linux(tmp_path):
    outfile = tmp_path / "linux.yml"

    generate_gha.build_linux_pipeline(
        [[ ["ros-demo-a"] ], [ ["ros-demo-b"] ]],
        "buildbranch_linux",
        outfile=str(outfile),
        pipeline_name="build_linux64",
        publish_mode="platform-finalize",
    )

    data = _load_yaml(outfile)
    jobs = data["jobs"]

    build_job = jobs["stage_0_job_0"]
    assert build_job["steps"][1]["env"]["VINCA_SKIP_UPLOAD"] == "1"
    assert build_job["steps"][2]["uses"] == "actions/upload-artifact@v6"
    assert build_job["steps"][2]["with"]["path"] == "~/conda-bld/linux-64"

    publish_job = jobs["publish_linux_64"]
    assert publish_job["needs"] == ["stage_1_job_1"]
    assert publish_job["steps"][2]["uses"] == "actions/download-artifact@v5"
    assert publish_job["steps"][3]["name"] == "Publish built packages for linux-64"


def test_generate_gha_unix_artifact_upload_uses_tilde_home_path():
    assert generate_gha.get_build_artifact_path("linux-64") == "~/conda-bld/linux-64"
    assert generate_gha.get_build_artifact_path("osx-64") == "~/conda-bld/osx-64"


def test_generate_gha_platform_finalize_windows(tmp_path):
    outfile = tmp_path / "win.yml"

    generate_gha.build_win_pipeline(
        [[["ros-demo-a"]]],
        "buildbranch_win",
        outfile=str(outfile),
        publish_mode="platform-finalize",
    )

    data = _load_yaml(outfile)
    jobs = data["jobs"]

    build_job = jobs["stage_0_job_0"]
    assert build_job["steps"][4]["env"]["VINCA_SKIP_UPLOAD"] == "1"
    assert build_job["steps"][5]["uses"] == "actions/upload-artifact@v6"
    assert "publish_win_64" in jobs


def test_generate_azure_platform_finalize_linux(tmp_path):
    outfile = tmp_path / "linux.yml"

    generate_azure.build_linux_pipeline(
        [[["ros-demo-a"]]],
        "buildbranch_linux",
        outfile=str(outfile),
        target="linux-64",
        publish_mode="platform-finalize",
    )

    data = _load_yaml(outfile)
    stages = data["stages"]

    build_stage = stages[0]
    build_job = build_stage["jobs"][0]
    assert build_job["steps"][0]["env"]["VINCA_SKIP_UPLOAD"] == "1"
    assert build_job["steps"][2]["task"] == "PublishPipelineArtifact@1"

    publish_stage = stages[1]
    assert publish_stage["stage"] == "publish_linux_64"
    assert publish_stage["dependsOn"] == ["stage_0"]


def test_generate_azure_platform_finalize_windows(tmp_path):
    outfile = tmp_path / "win.yml"

    generate_azure.build_win_pipeline(
        [[["ros-demo-a"]]],
        "buildbranch_win",
        outfile=str(outfile),
        publish_mode="platform-finalize",
    )

    data = _load_yaml(outfile)
    stages = data["stages"]

    build_job = stages[0]["jobs"][0]
    assert build_job["steps"][5]["env"]["VINCA_SKIP_UPLOAD"] == "1"
    assert build_job["steps"][7]["task"] == "PublishPipelineArtifact@1"

    publish_stage = stages[1]
    assert publish_stage["stage"] == "publish_win_64"
    assert publish_stage["jobs"][0]["steps"][0]["task"] == "DownloadPipelineArtifact@2"
