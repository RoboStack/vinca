"""Tests for planning the rebuild after a pinning change."""

import json

from vinca import rebuild
from vinca.rebuild import changed_pins, plan


def test_changed_pins_compares_values_and_pin_run_as_build():
    old = {
        "libboost_devel": ["1.88"],
        "pcl": ["1.15"],
        "eigen": ["3.4"],
        "zip_keys": [["a", "b"]],
        "pin_run_as_build": {"eigen": {"max_pin": "x.x"}},
    }
    new = {
        "libboost_devel": ["1.90"],
        "pcl": ["1.15"],
        "eigen": ["3.4"],
        "zip_keys": [["a", "c"]],
        "pin_run_as_build": {"eigen": {"max_pin": "x"}},
        "libprotobuf": ["7.35"],
    }

    changed = changed_pins(old, new)

    assert set(changed) == {"libboost-devel", "eigen", "libprotobuf"}
    assert changed["libboost-devel"] == (["1.88"], ["1.90"])
    assert changed["libprotobuf"] == (None, ["7.35"])


REQUIREMENTS = {
    "uses_boost": {
        "build": ["cmake"],
        "host": ["libboost-devel", "ros2-base"],
        "run": [],
    },
    "base": {"build": [], "host": ["eigen"], "run": []},
    "middle": {"build": [], "host": ["ros2-uses-boost"], "run": []},
    "top": {"build": [], "host": [], "run": [{"if": "linux", "then": ["ros2-middle"]}]},
    "unrelated": {"build": [], "host": ["ros2-base", "pcl"], "run": []},
}


def test_plan_rebuilds_direct_users_and_their_dependents():
    result = plan(REQUIREMENTS, ["libboost_devel"], "ros2")

    assert result == {
        "uses_boost": "uses libboost-devel",
        "middle": "depends on uses_boost",
        "top": "depends on middle",
    }


def test_plan_without_changes_rebuilds_nothing():
    assert plan(REQUIREMENTS, [], "ros2") == {}


def test_plan_follows_the_package_prefix():
    requirements = {
        "a": {"host": ["libfoo"]},
        "b": {"run": ["ros-humble-a"]},
    }

    assert plan(requirements, ["libfoo"], "ros-humble") == {
        "a": "uses libfoo",
        "b": "depends on a",
    }


def test_main_without_changed_pins_needs_no_recipes(tmp_path):
    cbc = tmp_path / "conda_build_config.yaml"
    cbc.write_text("pcl:\n  - '1.15'\n")
    out = tmp_path / "plan.json"

    assert rebuild.main(["--old", str(cbc), "--new", str(cbc), "--json", str(out)]) == 0
    assert json.loads(out.read_text()) == {
        "changed_pins": {},
        "packages": None,
        "rebuild": {},
    }
