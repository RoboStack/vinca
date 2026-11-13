import functools
import json
import socket
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import pytest
from rattler.exceptions import FetchRepoDataError

from vinca import utils
from vinca.schemas import PkgAdditionalInfo, VincaConfig
from vinca.utils import (
    CyclicTestRequirement,
    add_test_requirements,
    build_requirement_graph,
    extract_dependency_names,
    get_channel_records,
    get_pkg_additional_info,
    get_pkg_build_number,
)


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


@pytest.fixture
def channel(tmp_path, monkeypatch):
    monkeypatch.setattr(utils, "REPODATA_CACHE", tmp_path / "cache")
    return tmp_path / "channel"


@pytest.fixture
def served_channel(channel):
    channel.mkdir()
    handler = functools.partial(SimpleHTTPRequestHandler, directory=channel)
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield channel, f"http://127.0.0.1:{server.server_port}/"
    server.shutdown()
    server.server_close()


def _write_repodata(channel, subdir, content):
    (channel / subdir).mkdir(parents=True)
    (channel / subdir / "repodata.json").write_text(content)


def _repodata(*records):
    return json.dumps(
        {
            "packages": {},
            "packages.conda": {
                f"{r['name']}-{r['version']}-{r['build']}.conda": r for r in records
            },
        }
    )


FOO = {"name": "ros2-foo", "version": "1.0", "build": "h1_31", "build_number": 31}


def _names_and_builds(records):
    return [(r["name"], r["version"], r["build"], r["build_number"]) for r in records]


def test_get_channel_records_of_a_local_channel(channel):
    _write_repodata(channel, "linux-64", _repodata(FOO))

    for location in (str(channel), channel.as_uri()):
        records = get_channel_records(location, "linux-64")
        assert _names_and_builds(records) == [("ros2-foo", "1.0", "h1_31", 31)]
        assert records[0]["fn"] == "ros2-foo-1.0-h1_31.conda"


def test_get_channel_records_of_a_remote_channel(served_channel):
    channel, url = served_channel
    _write_repodata(channel, "linux-64", _repodata(FOO))

    records = get_channel_records(url, "linux-64")
    assert _names_and_builds(records) == [("ros2-foo", "1.0", "h1_31", 31)]


def test_get_channel_records_returns_no_records_for_missing_local_repodata(channel):
    assert get_channel_records(str(channel), "osx-arm64") == []


def test_get_channel_records_returns_no_records_for_missing_remote_repodata(
    served_channel,
):
    _, url = served_channel
    assert get_channel_records(url, "osx-arm64") == []


@pytest.mark.parametrize("content", ["Not Found", ""])
def test_get_channel_records_returns_no_records_for_invalid_remote_repodata(
    served_channel, content
):
    channel, url = served_channel
    _write_repodata(channel, "osx-arm64", content)
    assert get_channel_records(url, "osx-arm64") == []


def test_get_channel_records_raises_when_the_channel_is_unreachable(channel):
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]

    with pytest.raises(FetchRepoDataError):
        get_channel_records(f"http://127.0.0.1:{port}/", "linux-64")


ADDITIONAL_INFO_CONF = VincaConfig(
    ros_distro="humble",
    pkg_additional_info={
        "ros2_pulse": {"build_number": 7, "additional_cmake_args": "-DX=1"},
        "rtabmap_odom": {"build_number": 5},
    },
)


@pytest.mark.parametrize(
    "name", ["ros2_pulse", "ros2-ros2-pulse", "ros-humble-ros2-pulse"]
)
def test_additional_info_of_a_ros2_prefixed_package(name):
    # a ROS name starting with ros2_/ros_ was normalized twice (ros2_pulse -> pulse)
    assert get_pkg_additional_info(name, ADDITIONAL_INFO_CONF) == PkgAdditionalInfo(
        build_number=7, additional_cmake_args="-DX=1"
    )
    assert get_pkg_build_number(0, name, ADDITIONAL_INFO_CONF) == 7


@pytest.mark.parametrize(
    "name", ["rtabmap_odom", "ros2-rtabmap-odom", "ros-humble-rtabmap-odom"]
)
def test_additional_info_by_ros_or_conda_name(name):
    assert get_pkg_build_number(0, name, ADDITIONAL_INFO_CONF) == 5


def test_additional_info_of_a_package_without_an_entry():
    assert (
        get_pkg_additional_info("ros2-rclcpp", ADDITIONAL_INFO_CONF)
        == PkgAdditionalInfo()
    )
    assert (
        get_pkg_additional_info("rclcpp", VincaConfig(ros_distro="humble"))
        == PkgAdditionalInfo()
    )
