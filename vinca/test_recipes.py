import pytest

from vinca import recipes
from vinca.recipes import _dummy_constraint, generate_output

PACKAGE_XML = """<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>1.2.3</version>
  <description>Description of {name}.</description>
  <maintainer email="maintainer@example.com">Maintainer</maintainer>
  <license>Apache-2.0</license>
  <url type="website">https://example.org/{name}</url>
  <url type="repository">https://github.com/example/{name}</url>
{depends}
  <export><build_type>{build_type}</build_type></export>
</package>
"""

UNRESOLVABLE = {"missing_tool", "missing_dep"}

SYSTEM_PACKAGES = {
    "python": ["python"],
    "python-setuptools": ["setuptools"],
    "cmake": ["cmake"],
    "git": ["git"],
    "doxygen": ["doxygen"],
    "cyclonedds": ["ros2-cyclonedds"],
    "mimick_vendor": ["ros2-mimick-vendor"],
    "pybind11_vendor": ["ros2-pybind11-vendor"],
    "rosidl_default_generators": ["ros2-rosidl-default-generators"],
}


def package_xml(name, build_type="ament_cmake", depends=()):
    body = "\n".join(f"  <{tag}>{value}</{tag}>" for tag, value in depends)
    return PACKAGE_XML.format(name=name, depends=body, build_type=build_type)


class FakeDistro:
    name = "rolling"

    def __init__(self, xml_by_name, ros1=False):
        self._xml_by_name = xml_by_name
        self._ros1 = ros1

    def get_release_package_xml(self, name):
        return self._xml_by_name.get(name)

    def check_ros1(self):
        return self._ros1

    def get_package_prefix(self):
        return "ros2"

    def get_legacy_package_prefix(self):
        return "ros-rolling"


def fake_resolve(name, vinca_conf, distro, is_rundep=False):
    if name in UNRESOLVABLE:
        return []
    if name in SYSTEM_PACKAGES:
        return list(SYSTEM_PACKAGES[name])
    return [f"ros2-{name.replace('_', '-')}"]


@pytest.fixture(autouse=True)
def stub_resolve(monkeypatch):
    monkeypatch.setattr(recipes, "resolve_pkgname", fake_resolve)


def make_config(name, **overrides):
    config = {
        "_selected_pkgs": {name},
        "_pkg_additional_info": {},
        "depmods": {},
        "package_name_mode": "new",
        "build_number": 0,
    }
    config.update(overrides)
    return config


def build(
    name,
    depends=(),
    build_type="ament_cmake",
    unsatisfied=None,
    dependencies_only=False,
    **overrides,
):
    distro = FakeDistro({name: package_xml(name, build_type, depends)})
    return generate_output(
        name,
        make_config(name, **overrides),
        distro,
        "1.2.3",
        unsatisfied=unsatisfied,
        dependencies_only=dependencies_only,
    )


def test_generate_output_produces_a_complete_recipe():
    output = build("demo", depends=[("build_depend", "rclcpp")])

    assert output == {
        "package": {"name": "ros2-demo", "version": "1.2.3"},
        "requirements": {
            "build": [
                "${{ compiler('cxx') }}",
                "${{ compiler('c') }}",
                {
                    "if": "target_platform!='emscripten-wasm32'",
                    "then": ["${{ stdlib('c') }}"],
                },
                "ninja",
                "python",
                "setuptools",
                "git",
                "git-lfs",
                {"if": "unix", "then": ["patch", "make", "coreutils"]},
                {"if": "win", "then": ["m2-patch"]},
                {"if": "osx", "then": ["tapi"]},
                {"if": "build_platform != target_platform", "then": ["pkg-config"]},
                "cmake",
                "cython",
                {
                    "if": "build_platform != target_platform",
                    "then": ["python", "cross-python_${{ target_platform }}", "numpy"],
                },
                {
                    "if": "osx",
                    "then": [
                        "clang-tools ${{ (cxx_compiler_version ~ '.*') if "
                        "cxx_compiler_version is defined else '*' }}"
                    ],
                },
            ],
            "host": [
                {"if": "build_platform == target_platform", "then": ["pkg-config"]},
                "numpy",
                "pip",
                "python",
                "ros2-rclcpp",
                "ros2-ros-environment",
                "ros2-ros-workspace",
            ],
            "run": ["python", "ros2-ros-workspace"],
        },
        "build": {
            "script": "${{ '$RECIPE_DIR/build_ament_cmake.sh' if unix or wasm32 "
            "else '%RECIPE_DIR%\\\\bld_ament_cmake.bat' }}"
        },
        "about": {
            "homepage": "https://example.org/demo",
            "repository": "https://github.com/example/demo",
            "license": "Apache-2.0",
            "summary": "Description of demo.",
        },
    }


def test_cmake_is_build_only_on_emscripten():
    output = build("demo", depends=[("exec_depend", "cmake")])

    assert {
        "if": "target_platform != 'emscripten-wasm32'",
        "then": ["cmake"],
    } in output["requirements"]["run"]
    assert "cmake" not in output["requirements"]["run"]
    assert "cmake" not in output["requirements"]["host"]


def test_every_duplicate_cmake_is_replaced_by_the_selector():
    # catkin_pkg reports run_depends and exec_depends separately, so a single
    # <exec_depend>cmake</exec_depend> already yields two entries. Every one of them
    # has to be replaced, otherwise an unconditional cmake survives alongside the
    # selector and defeats it.
    output = build(
        "demo",
        depends=[("exec_depend", "cmake"), ("build_export_depend", "cmake")],
    )

    assert output["requirements"]["run"].count("cmake") == 0
    assert (
        output["requirements"]["run"].count(
            {"if": "target_platform != 'emscripten-wasm32'", "then": ["cmake"]}
        )
        == 1
    )


def test_mimick_vendor_moves_from_host_to_build():
    output = build("demo", depends=[("build_depend", "mimick_vendor")])

    assert "ros2-mimick-vendor" not in output["requirements"]["host"]
    assert {
        "if": "target_platform != 'emscripten-wasm32'",
        "then": ["ros2-mimick-vendor"],
    } in output["requirements"]["build"]


def test_every_duplicate_mimick_vendor_is_moved_out_of_host():
    output = build(
        "demo",
        depends=[
            ("build_depend", "mimick_vendor"),
            ("test_depend", "mimick_vendor"),
            ("build_export_depend", "mimick_vendor"),
        ],
    )

    assert "ros2-mimick-vendor" not in output["requirements"]["host"]


def test_rosidl_generators_are_added_to_build_for_emscripten():
    output = build("demo", depends=[("build_depend", "rosidl_default_generators")])

    assert {
        "if": "target_platform == 'emscripten-wasm32'",
        "then": ["ros2-rosidl-default-generators"],
    } in output["requirements"]["build"]


def test_pybind11_vendor_pulls_in_pybind11():
    output = build("demo", depends=[("build_depend", "pybind11_vendor")])

    assert "pybind11" in output["requirements"]["host"]
    assert {
        "if": "build_platform != target_platform",
        "then": ["pybind11"],
    } in output["requirements"]["build"]


def test_host_only_tools_are_split_across_build_and_host():
    output = build("demo", depends=[("build_depend", "doxygen")])

    assert {
        "if": "build_platform == target_platform",
        "then": ["doxygen"],
    } in output["requirements"]["host"]
    assert {
        "if": "build_platform != target_platform",
        "then": ["doxygen"],
    } in output["requirements"]["build"]
    assert "doxygen" not in output["requirements"]["host"]


def test_cyclonedds_gets_a_build_platform_copy():
    output = build("demo", depends=[("build_depend", "cyclonedds")])

    assert {
        "if": "build_platform != target_platform",
        "then": ["ros2-cyclonedds"],
    } in output["requirements"]["build"]


def test_cyclonedds_copy_does_not_depend_on_build_tools_resolving():
    # The check is independent of the build-tool loop: a package can depend on
    # cyclonedds while none of its build tools resolve, and it still needs the
    # build-platform copy.
    output = build(
        "demo",
        depends=[
            ("buildtool_depend", "missing_tool"),
            ("build_depend", "cyclonedds"),
        ],
        unsatisfied=set(),
    )

    assert {
        "if": "build_platform != target_platform",
        "then": ["ros2-cyclonedds"],
    } in output["requirements"]["build"]


def test_build_tools_land_in_host_except_git():
    output = build("demo", depends=[("buildtool_depend", "ament_cmake")])

    assert "ros2-ament-cmake" in output["requirements"]["host"]


def test_ament_python_adds_setuptools_to_host():
    output = build("demo", build_type="ament_python")

    assert "setuptools" in output["requirements"]["host"]
    assert "build_ament_python.sh" in output["build"]["script"]


def test_unknown_build_type_is_skipped():
    assert build("demo", build_type="make") is None


def test_unresolvable_dependencies_are_recorded():
    unsatisfied = set()
    build(
        "demo",
        depends=[("buildtool_depend", "missing_tool"), ("build_depend", "missing_dep")],
        unsatisfied=unsatisfied,
    )

    assert unsatisfied == {"missing_tool", "missing_dep"}


def test_depmods_add_and_remove_dependencies():
    output = build(
        "demo",
        depends=[("build_depend", "rclcpp")],
        depmods={"demo": {"add_host": ["extra-host"], "remove_host": ["ros2-rclcpp"]}},
    )

    assert "extra-host" in output["requirements"]["host"]
    assert "ros2-rclcpp" not in output["requirements"]["host"]


def test_mutex_dependency_is_added_to_host_and_run():
    output = build("demo", mutex_package="ros2-distro-mutex 0.5.* rolling_*")

    assert "ros2-distro-mutex 0.5.* rolling_*" in output["requirements"]["host"]
    assert "ros2-distro-mutex 0.5.* rolling_*" in output["requirements"]["run"]


def test_ros1_skips_the_workspace_dependencies():
    distro = FakeDistro({"demo": package_xml("demo")}, ros1=True)
    output = generate_output("demo", make_config("demo"), distro, "1.2.3")

    assert "ros2-ros-workspace" not in output["requirements"]["run"]


def test_dependencies_only_returns_requirements_without_metadata():
    requirements = build("demo", dependencies_only=True)

    assert set(requirements) == {"build", "host", "run"}
    assert "about" not in requirements


def test_unselected_package_produces_no_output():
    distro = FakeDistro({"demo": package_xml("demo")})
    config = make_config("demo", _selected_pkgs=set())

    assert generate_output("demo", config, distro, "1.2.3") is None


def test_missing_release_manifest_produces_no_output():
    distro = FakeDistro({})

    assert generate_output("demo", make_config("demo"), distro, "1.2.3") is None


def test_dummy_metapackage_pins_its_dependency():
    output = build(
        "demo",
        _pkg_additional_info={
            "demo": {
                "generate_dummy_package_with_run_deps": {
                    "dep_name": "vendor-library",
                    "upper_bound": "x.x",
                }
            }
        },
    )

    assert output["package"] == {"name": "ros2-demo", "version": "1.2.3"}
    assert output["build"]["script"] == ""
    assert "vendor-library >=1.2.3, <1.3.0a0" in output["requirements"]["run"]


def test_dummy_constraint_uses_pin_depth_and_override_version():
    version, constraint = _dummy_constraint(
        {
            "dep_name": "vendor-library",
            "upper_bound": "x.x",
            "override_version": "2.4.1",
        },
        "1.0.0",
        "demo_vendor",
    )

    assert version == "2.4.1"
    assert constraint == "vendor-library >=2.4.1, <2.5.0a0"


@pytest.mark.parametrize(
    ("config", "message"),
    [
        ({"upper_bound": "x"}, "dep_name"),
        ({"dep_name": "vendor-library"}, "upper_bound.*max_pin"),
    ],
)
def test_dummy_constraint_validates_required_settings(config, message):
    with pytest.raises(RuntimeError, match=message):
        _dummy_constraint(config, "1.0.0", "demo_vendor")
