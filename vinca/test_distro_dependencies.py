from unittest.mock import Mock

from vinca.distro import Distro


def make_test_distro():
    distro = Distro.__new__(Distro)
    distro.snapshot = None
    distro.additional_packages_snapshot = None
    distro._depends_cache = {}
    distro._direct_depends_cache = {}
    return distro


def test_dependency_walk_reuses_direct_dependencies_across_roots():
    graph = {
        "app": {"common", "app-leaf"},
        "other": {"common"},
        "common": {"common-leaf"},
        "app-leaf": set(),
        "common-leaf": set(),
    }
    distro = make_test_distro()
    distro.check_package = lambda name: name in graph
    distro._walker = Mock()
    distro._walker.get_depends.side_effect = (
        lambda package, dependency_type, ros_packages_only: (
            graph[package] if dependency_type == "run" else set()
        )
    )

    assert distro.get_depends("app") == {"common", "app-leaf", "common-leaf"}
    assert distro.get_depends("other") == {"common", "common-leaf"}

    # Each package's seven dependency fields are read only once even though the
    # two roots share a transitive dependency closure.
    assert distro._walker.get_depends.call_count == 5 * 7


def test_dependency_walk_honors_ignored_packages():
    graph = {
        "app": {"included", "ignored"},
        "included": set(),
        "ignored": {"hidden"},
        "hidden": set(),
    }
    distro = make_test_distro()
    distro.check_package = lambda name: name in graph
    distro._walker = Mock()
    distro._walker.get_depends.side_effect = (
        lambda package, dependency_type, ros_packages_only: (
            graph[package] if dependency_type == "run" else set()
        )
    )

    assert distro.get_depends("app", ignore_pkgs={"ignored"}) == {"included"}


def test_dependency_walk_excludes_root_in_cycles():
    graph = {"a": {"b"}, "b": {"a"}}
    distro = make_test_distro()
    distro.check_package = lambda name: name in graph
    distro._get_direct_depends = lambda name: graph[name]

    assert distro.get_depends("a") == {"b"}


def test_dependency_walk_excludes_root_from_self_dependency():
    distro = make_test_distro()
    distro.check_package = lambda name: name == "a"
    distro._get_direct_depends = lambda name: {"a"}

    assert distro.get_depends("a") == set()


def test_dependency_walk_excludes_root_from_longer_cycle():
    graph = {"a": {"b"}, "b": {"c"}, "c": {"a"}}
    distro = make_test_distro()
    distro.check_package = lambda name: name in graph
    distro._get_direct_depends = lambda name: graph[name]

    assert distro.get_depends("a") == {"b", "c"}


def test_prefetch_additional_manifests_skips_archives():
    distro = Distro.__new__(Distro)
    distro.additional_packages_snapshot = {
        "one": {"url": "https://github.com/example/one.git", "rev": "abc"},
        "two": {"url": "https://gitlab.com/example/two", "tag": "1.0"},
        "archive": {"url": "https://example.com/source.tar.gz", "sha256": "abc"},
    }
    distro.get_package_xml_for_additional_package = Mock(return_value="<package />")

    distro.prefetch_additional_package_xml(max_workers=2)

    requested = {
        call.args[0]["url"]
        for call in distro.get_package_xml_for_additional_package.call_args_list
    }
    assert requested == {
        "https://github.com/example/one.git",
        "https://gitlab.com/example/two",
    }
