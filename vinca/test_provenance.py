"""Tests for package-selection provenance tracking and the generation summary.

These cover *why* a recipe gets generated: whether a package was requested
directly by the config and which already-selected packages depend on it.
"""

import vinca.main as m


class FakeDistro:
    """Minimal Distro stand-in for selection/provenance tests."""

    def __init__(self, depends, ros1=False):
        self._depends = depends
        self._ros1 = ros1

    def check_ros1(self):
        return self._ros1

    def check_package(self, pkg):
        return True

    def get_depends(self, pkg, ignore_pkgs=None):
        deps = set(self._depends.get(pkg, set()))
        if ignore_pkgs:
            deps -= set(ignore_pkgs)
        return deps


def test_provenance_requested_and_required_by():
    distro = FakeDistro(
        {"app": {"libA", "libB"}, "libA": {"libcommon"}, "libB": {"libcommon"}},
        ros1=True,  # keep ROS2 auto-injection out of this assertion
    )
    conf = {"packages_select_by_deps": ["app", "libA"], "packages_skip_by_deps": None}

    selected = m.get_selected_packages(distro, conf)

    assert set(selected) == {"app", "libA", "libB", "libcommon"}

    prov = conf["_pkg_provenance"]
    # both seeds are recorded as requested by config
    assert prov["requested_by_config"] == {"app", "libA"}
    # libA is requested *and* depended on by app
    assert prov["required_by"]["libA"] == {"app"}
    # transitive dep pulled in by the requested seeds
    assert prov["required_by"]["libcommon"] == {"libA"}
    assert prov["required_by"]["libB"] == {"app"}
    # purely-requested package has no reverse-dep entry
    assert "app" not in prov["required_by"]


def test_provenance_skip_by_deps_excludes_package():
    distro = FakeDistro({"app": {"libA", "skipme"}}, ros1=True)
    conf = {
        "packages_select_by_deps": ["app"],
        "packages_skip_by_deps": ["skipme"],
    }

    selected = m.get_selected_packages(distro, conf)

    assert "skipme" not in selected
    assert "libA" in selected


def test_ros2_workspace_auto_injected_with_reason():
    distro = FakeDistro({"app": set()}, ros1=False)
    conf = {"packages_select_by_deps": ["app"], "packages_skip_by_deps": None}

    selected = m.get_selected_packages(distro, conf)

    assert "ros_workspace" in selected
    assert "ros_environment" in selected
    prov = conf["_pkg_provenance"]
    assert prov["required_by"]["ros_workspace"] == {"(automatic ROS2 dependency)"}


def test_generation_summary_output(monkeypatch, capsys):
    distro = FakeDistro(
        {"app": {"libA", "libB"}, "libA": {"libcommon"}, "libB": {"libcommon"}},
        ros1=True,
    )
    conf = {"packages_select_by_deps": ["app", "libA"], "packages_skip_by_deps": None}
    conf["_selected_pkgs"] = m.get_selected_packages(distro, conf)

    monkeypatch.setattr(
        m,
        "resolve_pkgname",
        lambda shortname, vinca_conf, distro, is_rundep=False: [
            "ros-humble-" + shortname.replace("_", "-")
        ],
    )
    monkeypatch.setattr(m, "distro", distro, raising=False)
    # keep the rich table on one wide line so substrings are not wrapped
    monkeypatch.setenv("COLUMNS", "200")

    # pretend a recipe was generated for everything except libB, plus a mutex
    outputs = [
        {"package": {"name": "ros-humble-" + s.replace("_", "-")}}
        for s in conf["_selected_pkgs"]
        if s != "libB"
    ]
    outputs.append({"package": {"name": "ros-humble-ros2-mutex"}})

    m.print_generation_summary(distro, conf, outputs)
    out = capsys.readouterr().out

    assert "Generated recipes and why they were selected" in out
    assert "Requested by config" in out  # table header
    assert "Depended on by" in out  # table header
    assert "ros-humble-app" in out
    assert "ros-humble-libcommon" in out
    assert "ros-humble-ros2-mutex" in out
    assert "auxiliary" in out  # mutex marked as auxiliary
    assert "ros-humble-libB" not in out  # not generated -> not listed
    # app, libA, libcommon (libB excluded) + mutex auxiliary
    assert "Total generated recipes: 4" in out
