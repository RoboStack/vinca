from pathlib import Path

from ruamel.yaml import YAML

from vinca.schemas import VincaConfig
from vinca.template import write_recipe


class FakeDistro:
    def get_package_prefix(self):
        return "ros2"

    def get_legacy_package_prefix(self):
        return "ros-jazzy"


def config(variants_mode):
    return VincaConfig(
        ros_distro="jazzy",
        package_name_mode="legacy",
        variants_mode=variants_mode,
        build_number=0,
        variant_config={"python": ["3.12"], "unused": ["1"]},
    )


def output():
    return {
        "package": {"name": "ros-jazzy-demo", "version": "1.0"},
        "requirements": {"host": ["python"], "run": ["python"]},
        "build": {"script": ""},
    }


def test_write_recipe_emits_pruned_variants_in_local_mode(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)

    write_recipe({}, [output()], config("local"), FakeDistro(), single_file=False)

    variants_path = Path("recipes/ros-jazzy-demo/variants.yaml")
    assert YAML(typ="safe").load(variants_path) == {"python": ["3.12"]}


def test_write_recipe_appends_test_file_to_generated_tests(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    test_file = tmp_path / "ros-jazzy-demo.yaml"
    test_file.write_text("tests:\n  - script:\n      - demo --help\n")
    conf = config("global")
    conf.tests = {"ros-jazzy-demo": test_file}
    generated = {"package_contents": {"files": {"not_exists": ["**/*.pyd"]}}}

    write_recipe(
        {}, [{**output(), "tests": [generated]}], conf, FakeDistro(), single_file=False
    )

    recipe = YAML(typ="safe").load(Path("recipes/ros-jazzy-demo/recipe.yaml"))
    assert recipe["tests"] == [generated, {"script": ["demo --help"]}]


def test_write_recipe_does_not_emit_variants_in_global_mode(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)

    write_recipe({}, [output()], config("global"), FakeDistro(), single_file=False)

    assert not Path("recipes/ros-jazzy-demo/variants.yaml").exists()
