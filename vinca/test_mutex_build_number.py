"""mutex_package.build_number: a new build of the mutex at the same version."""

from unittest.mock import Mock

import yaml

from vinca.main import expected_build_number
from vinca.mutex import generate_mutex_package_recipe
from vinca.template import write_recipe


def _conf(**extra):
    conf = {
        "ros_distro": "lyrical",
        "build_number": 27,
        "mutex_package": {
            "name": "ros2-distro-mutex",
            "version": "0.21.0",
            "upper_bound": "x.x",
            "run_constraints": [],
            "build_number": 28,
        },
        "_pkg_additional_info": {"rclcpp": {"build_number": 30}},
        "_tests": {},
    }
    conf.update(extra)
    return conf


def test_expected_build_number_of_the_mutex_is_its_own():
    assert expected_build_number(27, "ros2-distro-mutex", _conf()) == 28


def test_expected_build_number_of_other_packages():
    conf = _conf()
    assert expected_build_number(27, "ros2-rclcpp", conf) == 30
    assert expected_build_number(27, "ros2-std-msgs", conf) == 27


def test_mutex_without_own_build_number_uses_the_distribution_one():
    conf = _conf()
    del conf["mutex_package"]["build_number"]
    assert expected_build_number(27, "ros2-distro-mutex", conf) == 27


def test_mutex_recipe_keeps_its_build_number(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    conf = _conf()
    distro = Mock()
    distro.name = "lyrical"
    recipe = generate_mutex_package_recipe(conf, distro)
    write_recipe({}, [recipe], conf, distro, single_file=False)
    meta = yaml.safe_load(
        (tmp_path / "recipes" / "ros2-distro-mutex" / "recipe.yaml").read_text()
    )
    assert meta["build"]["number"] == 28
    assert meta["build"]["string"] == "lyrical_28"
    assert meta["requirements"]["run_constraints"] == []
