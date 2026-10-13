from vinca.utils import built_for_distro


def test_built_for_distro_by_mutex_dependency():
    lyrical = {
        "name": "ros2-plotjuggler",
        "build": "np2py314h1_30",
        "depends": ["python", "ros2-distro-mutex 0.22.* lyrical_*"],
    }
    rolling = {
        "name": "ros2-plotjuggler",
        "build": "np2py314h2_30",
        "depends": ["python", "ros2-distro-mutex 0.22.* rolling_*"],
    }
    assert built_for_distro(lyrical, "lyrical")
    assert not built_for_distro(rolling, "lyrical")


def test_built_for_distro_mutex_and_unmarked_packages():
    assert built_for_distro(
        {"name": "ros2-distro-mutex", "build": "lyrical_28"}, "lyrical"
    )
    assert not built_for_distro(
        {"name": "ros2-distro-mutex", "build": "rolling_28"}, "lyrical"
    )
    # no mutex dependency (e.g. older or non-ROS records): as before
    assert built_for_distro(
        {"name": "ros-lyrical-foo", "build": "h0_1", "depends": ["ros2-foo"]}, "lyrical"
    )
