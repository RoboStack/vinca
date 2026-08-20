import pytest

from vinca.mutex import (
    generate_mutex_package_recipe,
    get_mutex_package_dependency,
    parse_mutex_package_config,
    should_skip_mutex_package,
)


class Distro:
    name = "jazzy"


def mutex_config(**overrides):
    config = {
        "mutex_package": {
            "name": "ros2-distro-mutex",
            "version": "1.2.0",
            "upper_bound": "x.x",
            "run_constraints": ["ros2-distro-mutex 1.2.*"],
        },
        "build_number": 3,
        "skip_built_packages": [],
    }
    config.update(overrides)
    return config


def test_parse_mutex_config_adds_default_build_number():
    parsed = parse_mutex_package_config(mutex_config())

    assert parsed["build_number"] == 3


def test_parse_mutex_config_reports_missing_fields():
    with pytest.raises(ValueError, match="upper_bound"):
        parse_mutex_package_config({"mutex_package": {"name": "mutex"}})


def test_legacy_mutex_is_used_as_dependency_without_generating_recipe():
    config = {"mutex_package": "ros2-distro-mutex 1.*"}

    assert get_mutex_package_dependency(config, Distro()) == "ros2-distro-mutex 1.*"
    assert generate_mutex_package_recipe(config, Distro()) is None


def test_dictionary_mutex_dependency_and_recipe():
    config = mutex_config()

    assert (
        get_mutex_package_dependency(config, Distro())
        == "ros2-distro-mutex 1.2.* jazzy_*"
    )
    recipe = generate_mutex_package_recipe(config, Distro())
    assert recipe["build"] == {"number": 3, "string": "jazzy_3", "script": ""}
    assert (
        "pin_subpackage('ros2-distro-mutex', upper_bound='x.x')"
        in recipe["requirements"]["run_exports"]["weak"][0]
    )


def test_skip_mutex_uses_name_or_name_and_version():
    assert should_skip_mutex_package({"skip_built_packages": ["mutex"]}, "mutex", "1.0")
    assert should_skip_mutex_package(
        {
            "trigger_new_versions": True,
            "skip_built_packages": [("mutex", "1.0")],
        },
        "mutex",
        "1.0",
    )
