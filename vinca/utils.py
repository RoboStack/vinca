import asyncio
import json
import tempfile
from pathlib import Path
from urllib.parse import urlsplit

import networkx as nx
import rattler
from rattler.exceptions import FetchRepoDataError
from ruamel.yaml.representer import SafeRepresenter

REPODATA_CACHE = Path(tempfile.gettempdir()) / "vinca-repodata"


class folded_unicode(str):
    pass


class literal_unicode(str):
    pass


def folded_unicode_representer(dumper, data):
    return dumper.represent_scalar("tag:yaml.org,2002:str", str(data), style=">")


def literal_unicode_representer(dumper, data):
    return dumper.represent_scalar("tag:yaml.org,2002:str", str(data), style="|")


class NoAliasRepresenter(SafeRepresenter):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.sort_base_mapping_type_on_output = False

    def ignore_aliases(self, data) -> bool:
        return True


NoAliasRepresenter.add_representer(folded_unicode, folded_unicode_representer)
NoAliasRepresenter.add_representer(literal_unicode, literal_unicode_representer)


def _channel(location) -> rattler.Channel:
    # rattler reads a bare name as an anaconda.org channel; a single-letter scheme
    # is a Windows drive
    if len(urlsplit(location).scheme) > 1:
        return rattler.Channel(location)
    return rattler.Channel(str(Path(location).resolve()))


def get_channel_records(channel, platform):
    """The package records of *channel*, a URL or local path, for *platform*, as
    repodata.json records; none if the channel has no valid repodata for it."""
    try:
        [repodata] = asyncio.run(
            rattler.fetch_repo_data(
                channels=[_channel(channel)],
                platforms=[rattler.Subdir(platform)],
                cache_path=REPODATA_CACHE,
                callback=None,
            )
        )
        records = repodata.load_all_records()
    except FetchRepoDataError as exc:
        # network failures raise the same type
        if not str(exc).startswith("repodata not found"):
            raise
        print(
            f"No {platform} repodata found at {channel}, assuming no existing packages"
        )
        return []
    except OSError:
        print(
            f"No valid {platform} repodata found at {channel}, "
            "assuming no existing packages"
        )
        return []
    return [json.loads(record.to_json()) for record in records]


def ensure_name_is_without_distro_prefix_and_with_underscores(name, vinca_conf):
    """Normalize new and legacy ROS package names to their ROS package name."""
    newname = name.replace("-", "_")
    prefixes = ["ros2_"]
    if ros_distro := vinca_conf.get("ros_distro"):
        prefixes.append(f"ros_{ros_distro}_")
    prefixes.append("ros_")

    for prefix in prefixes:
        if newname.startswith(prefix):
            return newname[len(prefix) :]

    return newname


def extract_dependency_names(requirements) -> list[str]:
    """Extract dependency names from strings and conditional requirement structures."""
    names = []

    def visit(value) -> None:
        if isinstance(value, str):
            if value:
                names.append(value.split()[0])
        elif isinstance(value, dict):
            for key, nested_value in value.items():
                if key != "if":
                    visit(nested_value)
        elif isinstance(value, (list, tuple)):
            for nested_value in value:
                visit(nested_value)

    visit(requirements)
    return list(dict.fromkeys(names))


def add_test_requirements(requirements, metas):
    """Fold the test-only requirements into the per-package requirement lists.

    A requirement that only appears under ``tests`` still has to be installable
    when the test environment is solved, so it has to be built before the
    package that tests against it. The requirements are read from the generated
    recipes because those already have the legacy compatibility outputs (which
    never carry tests) filtered out.

    Returns the test-only requirements per package, so that the caller can treat
    them differently from the real dependencies when building the graph.
    """
    test_requirements = {}
    for pkg in metas:
        pkg_name = pkg["package"]["name"]
        if pkg_name not in requirements:
            continue
        test_reqs = []
        for test in pkg.get("tests") or []:
            if not isinstance(test, dict):
                continue
            test_section = test.get("requirements") or {}
            test_reqs += test_section.get("run", []) + test_section.get("build", [])
        test_reqs = [
            r
            for r in extract_dependency_names(test_reqs)
            if r not in requirements[pkg_name]
        ]
        if test_reqs:
            test_requirements[pkg_name] = test_reqs
            requirements[pkg_name] = requirements[pkg_name] + test_reqs
    return test_requirements


class CyclicTestRequirement(Exception):
    """A test requirement cannot be built before the package that tests it."""


def build_requirement_graph(requirements, test_requirements):
    """Build the dependency graph the build stages are derived from.

    The real dependency edges are added first, so that a test requirement can
    never reorder a real dependency. The test-only edges are added afterwards
    and a cycle is reported as an error: the build and the test of a package
    happen in the same job, so a package that is only built after the package
    testing against it can never be installed into that test environment. There
    is nothing this function could silently drop to make such a pipeline work --
    the recipe declares the requirement either way -- so the recipe has to be
    fixed instead.
    """
    G = nx.DiGraph()
    for pkg, reqs in requirements.items():
        G.add_node(pkg)
        for r in reqs:
            if r in test_requirements.get(pkg, ()):
                continue
            if r.startswith("ros-") or r.startswith("ros2-"):
                G.add_edge(pkg, r)

    for pkg, test_reqs in test_requirements.items():
        for r in test_reqs:
            if not (r.startswith("ros-") or r.startswith("ros2-")):
                continue
            G.add_edge(pkg, r)
            if not nx.is_directed_acyclic_graph(G):
                cycle = nx.find_cycle(G, source=pkg)
                path = " -> ".join([edge[0] for edge in cycle] + [cycle[-1][1]])
                raise CyclicTestRequirement(
                    f"The test requirement {r} of {pkg} closes the dependency "
                    f"cycle {path}, so {r} can only be built after {pkg} has "
                    f"already been built and tested. Either make {r} a real "
                    f"dependency of {pkg}, or move the test to a package that "
                    f"is built after both of them."
                )
    return G


def add_package_name_variants(entries, ros_distro) -> None:
    """Make package-specific config entries available under old and new names."""
    legacy_prefix = f"ros-{ros_distro}-"
    for name, value in list(entries.items()):
        if name.startswith(legacy_prefix):
            shortname = name[len(legacy_prefix) :]
        elif name.startswith("ros2-"):
            shortname = name[len("ros2-") :]
        elif name.startswith("ros-"):
            shortname = name[len("ros-") :]
        else:
            shortname = name

        for variant in (
            f"{legacy_prefix}{shortname}",
            f"ros-{shortname}",
            f"ros2-{shortname}",
        ):
            entries.setdefault(variant, value)


def get_pkg_additional_info(pkg_name, vinca_conf):
    """The pkg_additional_info.yaml entry of a package, given its ROS name (ros2_pulse)
    or its conda name (ros2-ros2-pulse, ros-humble-ros2-pulse). A ROS name is used as
    it is: normalizing it again would strip a ROS name's own ros2_/ros_ prefix."""
    pkg_additional_info_all = vinca_conf.get("_pkg_additional_info") or {}
    name = pkg_name.replace("-", "_")
    if name not in pkg_additional_info_all:
        name = ensure_name_is_without_distro_prefix_and_with_underscores(
            pkg_name, vinca_conf
        )
    return pkg_additional_info_all.get(name, {})


def get_pkg_build_number(default_build_number, pkg_name, vinca_conf):
    pkg_additional_info = get_pkg_additional_info(pkg_name, vinca_conf)
    return pkg_additional_info.get("build_number", default_build_number)


# Return true if the package is actually provided in conda-forge, and so we generate
# only a recipe with a run dependency on the conda forge package
def is_dummy_metapackage(pkg_name, vinca_conf) -> bool:
    pkg_additional_info = get_pkg_additional_info(pkg_name, vinca_conf)
    if pkg_additional_info.get("generate_dummy_package_with_run_deps"):
        return True
    else:
        return False


def built_for_distro(pkg: dict, ros_distro: str) -> bool:
    """Whether a repodata record was built for *ros_distro*: ROS 2 packages share their
    names across distributions (ros2-*), so a channel holding several of them must not
    make one distribution skip a package because another one has it. The distribution
    shows in the mutex dependency (``ros2-distro-mutex 0.22.* lyrical_*``) or, for the
    mutex itself, in its build string; records without either are kept."""
    if pkg.get("name", "").endswith("distro-mutex"):
        return pkg.get("build", "").startswith(f"{ros_distro}_")
    mutexes = [
        d for d in pkg.get("depends", []) if d.split(" ")[0].endswith("distro-mutex")
    ]
    if not mutexes:
        return True
    return any(f" {ros_distro}_" in d or d.endswith(f" {ros_distro}") for d in mutexes)
