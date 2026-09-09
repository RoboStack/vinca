import hashlib
import json
import os
import time

import networkx as nx
import requests
import yaml


class folded_unicode(str):
    pass


class literal_unicode(str):
    pass


def folded_unicode_representer(dumper, data):
    return dumper.represent_scalar("tag:yaml.org,2002:str", data, style=">")


def literal_unicode_representer(dumper, data):
    return dumper.represent_scalar("tag:yaml.org,2002:str", data, style="|")


yaml.add_representer(folded_unicode, folded_unicode_representer)
yaml.add_representer(literal_unicode, literal_unicode_representer)


class NoAliasDumper(yaml.SafeDumper):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.add_representer(folded_unicode, folded_unicode_representer)
        self.add_representer(literal_unicode, literal_unicode_representer)

    def ignore_aliases(self, data):
        return True


def get_repodata(url_or_path, platform=None):
    if platform:
        if not url_or_path.endswith("/"):
            url_or_path += "/"
        url_or_path += f"{platform}/repodata.json"

    if "://" not in url_or_path:
        if not os.path.exists(url_or_path):
            print(f"No repodata found at {url_or_path}, assuming no existing packages")
            return {"packages": {}, "packages.conda": {}}
        with open(url_or_path) as fi:
            return json.load(fi)
    print("Downloading repodata from ", url_or_path)

    m = hashlib.md5(url_or_path.encode("utf-8")).hexdigest()[:10]
    # print(tempfile.gettempdir())
    fn = f"vinca_{m}.json"
    if os.path.exists(fn):
        st = os.stat(fn)
        age = time.time() - st.st_mtime
        print(f"Found cached repodata, age: {age}")
        max_age = 100_000  # seconds == 27 hours
        if age < max_age:
            try:
                with open(fn) as fi:
                    return json.load(fi)
            except json.JSONDecodeError:
                print(f"Ignoring invalid cached repodata at {fn}")
                os.remove(fn)

    repodata = requests.get(url_or_path)
    if repodata.status_code == 404:
        print(f"No repodata found at {url_or_path}, assuming no existing packages")
        return {"packages": {}, "packages.conda": {}}
    repodata.raise_for_status()
    content = repodata.content
    if not content.strip():
        print(f"No repodata found at {url_or_path}, assuming no existing packages")
        return {"packages": {}, "packages.conda": {}}
    try:
        parsed_repodata = json.loads(content)
    except json.JSONDecodeError:
        print(
            f"No valid repodata found at {url_or_path}, assuming no existing packages"
        )
        return {"packages": {}, "packages.conda": {}}

    with open(fn, "w") as fcache:
        fcache.write(content.decode("utf-8"))
    return parsed_repodata


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


def extract_dependency_names(requirements):
    """Extract dependency names from strings and conditional requirement structures."""
    names = []

    def visit(value):
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


def add_package_name_variants(entries, ros_distro):
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
    normalized_name = ensure_name_is_without_distro_prefix_and_with_underscores(
        pkg_name, vinca_conf
    )
    pkg_additional_info_all = vinca_conf["_pkg_additional_info"]
    if pkg_additional_info_all is None:
        pkg_additional_info = {}
    else:
        pkg_additional_info = vinca_conf.get("_pkg_additional_info", {}).get(
            normalized_name, {}
        )
    return pkg_additional_info


def get_pkg_build_number(default_build_number, pkg_name, vinca_conf):
    pkg_additional_info = get_pkg_additional_info(pkg_name, vinca_conf)
    return pkg_additional_info.get("build_number", default_build_number)


# Return true if the package is actually provided in conda-forge, and so we generate
# only a recipe with a run dependency on the conda forge package
def is_dummy_metapackage(pkg_name, vinca_conf):
    pkg_additional_info = get_pkg_additional_info(pkg_name, vinca_conf)
    if pkg_additional_info.get("generate_dummy_package_with_run_deps"):
        return True
    else:
        return False
