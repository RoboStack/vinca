import json
from unittest.mock import Mock, patch

import pytest

from vinca.utils import (
    CyclicTestRequirement,
    add_test_requirements,
    build_requirement_graph,
    extract_dependency_names,
    get_repodata,
)

EMPTY_REPODATA = {"packages": {}, "packages.conda": {}}


def test_extract_dependency_names_handles_conditional_requirements():
    requirements = [
        "ros2-rclcpp >=1",
        {
            "if": "target_platform == 'emscripten-wasm32'",
            "then": ["ros2-rmw-wasm-cpp", "fmt"],
        },
        "ros2-rclcpp >=1",
    ]

    assert extract_dependency_names(requirements) == [
        "ros2-rclcpp",
        "ros2-rmw-wasm-cpp",
        "fmt",
    ]


def _meta(name, tests):
    return {"package": {"name": name}, "tests": tests}


def test_add_test_requirements_collects_test_only_requirements():
    requirements = {"ros2-pcl-conversions": ["ros2-rclcpp"], "ros2-ros2pkg": []}
    metas = [
        _meta(
            "ros2-pcl-conversions",
            [
                {
                    "script": ["ros2 pkg prefix pcl_conversions"],
                    "requirements": {"run": ["ros2-ros2pkg", "ros2-rclcpp"]},
                }
            ],
        )
    ]

    test_requirements = add_test_requirements(requirements, metas)

    # ros2-rclcpp is already a real dependency, so it is not reported again.
    assert test_requirements == {"ros2-pcl-conversions": ["ros2-ros2pkg"]}
    assert requirements["ros2-pcl-conversions"] == ["ros2-rclcpp", "ros2-ros2pkg"]


def test_add_test_requirements_ignores_recipes_without_tests():
    requirements = {"ros2-rclcpp": []}
    metas = [_meta("ros2-rclcpp", None), _meta("ros2-not-built", [])]

    assert add_test_requirements(requirements, metas) == {}
    assert requirements == {"ros2-rclcpp": []}


def test_build_requirement_graph_orders_test_requirements():
    requirements = {"ros2-pcl-conversions": ["ros2-ros2pkg"], "ros2-ros2pkg": []}
    test_requirements = {"ros2-pcl-conversions": ["ros2-ros2pkg"]}

    graph = build_requirement_graph(requirements, test_requirements)

    assert graph.has_edge("ros2-pcl-conversions", "ros2-ros2pkg")


def test_build_requirement_graph_rejects_cyclic_test_requirements():
    requirements = {"ros2-a": ["ros2-b"], "ros2-b": ["ros2-a"]}
    test_requirements = {"ros2-b": ["ros2-a"]}

    with pytest.raises(CyclicTestRequirement) as excinfo:
        build_requirement_graph(requirements, test_requirements)

    assert "ros2-a" in str(excinfo.value)
    assert "ros2-b" in str(excinfo.value)


def test_get_repodata_returns_empty_for_missing_local_repodata(tmp_path):
    assert get_repodata(str(tmp_path), "osx-arm64") == EMPTY_REPODATA


def test_get_repodata_returns_empty_for_missing_remote_repodata(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    response = Mock(status_code=404, content=b"Not Found")

    with patch("vinca.utils.requests.get", return_value=response):
        assert (
            get_repodata("https://example.com/channel", "osx-arm64") == EMPTY_REPODATA
        )


def test_get_repodata_returns_empty_for_non_json_remote_repodata(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    response = Mock(status_code=200, content=b"Not Found")
    response.raise_for_status.return_value = None

    with patch("vinca.utils.requests.get", return_value=response):
        assert (
            get_repodata("https://example.com/channel", "osx-arm64") == EMPTY_REPODATA
        )


def test_get_repodata_ignores_invalid_cached_repodata(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    url = "https://example.com/channel/osx-arm64/repodata.json"
    cache_name = "vinca_d7e1ca2423.json"
    (tmp_path / cache_name).write_text("Not Found")
    repodata = {"packages": {"pkg.tar.bz2": {"name": "pkg"}}, "packages.conda": {}}
    response = Mock(status_code=200, content=json.dumps(repodata).encode("utf-8"))
    response.raise_for_status.return_value = None

    with patch("vinca.utils.requests.get", return_value=response):
        assert get_repodata(url) == repodata
