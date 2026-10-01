from pathlib import Path

from ruamel.yaml import YAML

from vinca.template import write_recipe


class FakeDistro:
    def get_package_prefix(self):
        return "ros2"

    def get_legacy_package_prefix(self):
        return "ros-jazzy"


def config(variants_mode):
    return {
        "ros_distro": "jazzy",
        "package_name_mode": "legacy",
        "variants_mode": variants_mode,
        "build_number": 0,
        "_tests": {},
        "_test_folders": {},
        "_additional_packages_snapshot": {},
        "_pkg_additional_info": {},
        "_variant_config": {"python": ["3.12"], "unused": ["1"]},
    }


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


def test_write_recipe_does_not_emit_variants_in_global_mode(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)

    write_recipe({}, [output()], config("global"), FakeDistro(), single_file=False)

    assert not Path("recipes/ros-jazzy-demo/variants.yaml").exists()
