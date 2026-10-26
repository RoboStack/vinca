"""Tests for skipping recipes whose exact builds are already published."""

import shutil
from pathlib import Path

import pytest

from vinca import skip_existing
from vinca.skip_existing import (
    built_recipes,
    published_builds,
    remove_built_recipes,
    render_outputs,
)

LYRICAL_MUTEX = "ros2-distro-mutex 0.22.* lyrical_*"
ROLLING_MUTEX = "ros2-distro-mutex 0.22.* rolling_*"


def _record(name, version, build, mutex=LYRICAL_MUTEX):
    return {"name": name, "version": version, "build": build, "depends": [mutex]}


def test_published_builds_are_name_version_and_build_string():
    repodata = {
        "packages": {"a.tar.bz2": _record("ros2-rcl", "10.2.0", "hb2aaf94_30")},
        "packages.conda": {
            "b.conda": _record("ros2-rclpy", "10.0.1", "np2py314h40712d6_30")
        },
    }

    assert published_builds([repodata], "lyrical") == {
        ("ros2-rcl", "10.2.0", "hb2aaf94_30"),
        ("ros2-rclpy", "10.0.1", "np2py314h40712d6_30"),
    }


def test_published_builds_leave_out_other_distributions():
    repodata = {
        "packages.conda": {
            "a.conda": _record("ros2-rcl", "10.2.0", "h1_30", ROLLING_MUTEX)
        }
    }

    assert published_builds([repodata], "lyrical") == set()


def test_recipe_with_every_output_published_is_built():
    rendered = {"ros2-rcl": [("ros2-rcl", "10.2.0", "hb2aaf94_30")]}
    published = {("ros2-rcl", "10.2.0", "hb2aaf94_30")}

    assert built_recipes(rendered, published) == {"ros2-rcl"}


def test_recipe_with_a_missing_python_variant_is_kept():
    rendered = {
        "ros2-rclpy": [
            ("ros2-rclpy", "10.0.1", "np2py312hfd3a116_30"),
            ("ros2-rclpy", "10.0.1", "np2py314h40712d6_30"),
        ]
    }
    published = {("ros2-rclpy", "10.0.1", "np2py314h40712d6_30")}

    assert built_recipes(rendered, published) == set()


def test_recipe_with_a_new_build_number_is_kept():
    rendered = {"ros2-rcl": [("ros2-rcl", "10.2.0", "hb2aaf94_31")]}
    published = {("ros2-rcl", "10.2.0", "hb2aaf94_30")}

    assert built_recipes(rendered, published) == set()


def test_recipe_without_rendered_outputs_is_kept():
    assert built_recipes({"ros2-rcl": []}, set()) == set()


def test_remove_built_recipes_deletes_only_complete_ones(tmp_path, monkeypatch):
    for name in ("ros2-rcl", "ros2-rclpy"):
        (tmp_path / name).mkdir()
        (tmp_path / name / "recipe.yaml").write_text("")
    rendered = {
        "ros2-rcl": [("ros2-rcl", "10.2.0", "h1_30")],
        "ros2-rclpy": [
            ("ros2-rclpy", "10.0.1", "np2py312h2_30"),
            ("ros2-rclpy", "10.0.1", "np2py314h3_30"),
        ],
    }
    monkeypatch.setattr(skip_existing, "render_outputs", lambda *_args: rendered)
    published = {
        ("ros2-rcl", "10.2.0", "h1_30"),
        ("ros2-rclpy", "10.0.1", "np2py314h3_30"),
    }

    removed = remove_built_recipes(tmp_path, "linux-64", None, published)

    assert removed == {"ros2-rcl"}
    assert sorted(p.name for p in tmp_path.iterdir()) == ["ros2-rclpy"]


RECIPE = """\
package:
  name: {name}
  version: "1.0.0"
build:
  number: 30
requirements:
  host:
{host}
"""


@pytest.mark.skipif(shutil.which("rattler-build") is None, reason="needs rattler-build")
def test_render_outputs_gives_one_build_string_per_variant(tmp_path: Path):
    recipes = tmp_path / "recipes"
    for name, host in (("ros2-py", "    - python"), ("ros2-cpp", "    - zlib")):
        (recipes / name).mkdir(parents=True)
        (recipes / name / "recipe.yaml").write_text(RECIPE.format(name=name, host=host))
    variants = tmp_path / "conda_build_config.yaml"
    variants.write_text("python:\n  - 3.12.* *_cpython\n  - 3.14.* *_cp314\n")

    rendered = render_outputs(recipes, "linux-64", variants)

    assert sorted(b[2][:5] for b in rendered["ros2-py"]) == ["py312", "py314"]
    assert len(rendered["ros2-cpp"]) == 1
    assert "py" not in rendered["ros2-cpp"][0][2]
