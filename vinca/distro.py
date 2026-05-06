import os
import urllib.parse

import catkin_pkg.package
from rosdistro import get_cached_distribution, get_index, get_index_url
from rosdistro.dependency_walker import DependencyWalker
from rosdistro.manifest_provider import get_release_tag

from vinca import http as vinca_http
from vinca.pixi_manifest import parse_additional_manifest


class Distro(object):
    def __init__(
        self,
        distro_name,
        python_version=None,
        snapshot=None,
        additional_packages_snapshot=None,
    ):
        index = get_index(get_index_url())
        self._distro = get_cached_distribution(index, distro_name)
        self.distro_name = distro_name
        self.snapshot = snapshot
        self.additional_packages_snapshot = additional_packages_snapshot

        # set up ROS environments
        if python_version is None:
            python_version = index.distributions[distro_name]["python_version"]
        os.environ["ROS_PYTHON_VERSION"] = "{0}".format(python_version)
        os.environ["ROS_DISTRO"] = "{0}".format(distro_name)
        if "ROS_ROOT" in os.environ:
            os.environ.pop("ROS_ROOT")
        if "ROS_PACKAGE_PATH" in os.environ:
            os.environ.pop("ROS_PACKAGE_PATH")
        self._walker = DependencyWalker(
            self._distro, evaluate_condition_context=os.environ
        )

        # cache distribution type
        self._distribution_type = index.distributions[distro_name]["distribution_type"]
        self._python_version = index.distributions[distro_name]["python_version"]
        self.build_packages = set()

        # simple caches to avoid repeatedly fetching/processing the same data
        self._additional_xml_cache = {}
        self._depends_cache = {}

        os.environ["ROS_VERSION"] = "1" if self.check_ros1() else "2"

    @property
    def name(self):
        return self.distro_name

    def add_packages(self, packages):
        self.build_packages = set(packages)

    def get_depends(self, pkg, ignore_pkgs=None):
        dependencies = set()

        cache_key = (pkg, tuple(sorted(ignore_pkgs)) if ignore_pkgs else None)
        if cache_key in self._depends_cache:
            return set(self._depends_cache[cache_key])

        if not self.check_package(pkg):
            print(f"{pkg} not in available packages anymore")
            return dependencies

        # if pkg comes from additional_packages_snapshot, extract from its manifest
        if (
            self.additional_packages_snapshot
            and pkg in self.additional_packages_snapshot
        ):
            parsed = self.get_release_package(pkg)
            direct = {
                d.name
                for slot in (
                    parsed.build_depends,
                    parsed.buildtool_depends,
                    parsed.build_export_depends,
                    parsed.buildtool_export_depends,
                    parsed.exec_depends,
                    parsed.test_depends,
                )
                for d in slot
            }
            # add direct deps
            dependencies |= direct
            # recursively collect dependencies
            for dep in direct:
                if ignore_pkgs and dep in ignore_pkgs:
                    continue
                dependencies |= self.get_depends(dep, ignore_pkgs=ignore_pkgs)
            self._depends_cache[cache_key] = set(dependencies)
            return dependencies

        # If the package is from upstream rosdistro, use the walker to get dependencies
        dependencies |= self._walker.get_recursive_depends(
            pkg,
            [
                "buildtool",
                "buildtool_export",
                "build",
                "build_export",
                "run",
                "test",
                "exec",
            ],
            ros_packages_only=True,
            ignore_pkgs=ignore_pkgs,
        )
        self._depends_cache[cache_key] = set(dependencies)
        return dependencies

    def get_released_repo(self, pkg_name):
        if self.snapshot and pkg_name in self.snapshot:
            # In the case of snapshot, for rosdistro_additional_recipes
            # we also support a 'rev' field, so depending on what is available
            # we return either the tag or the rev, and the third argument is either 'rev' or 'tag'
            url = self.snapshot[pkg_name].get("url", None)
            if "tag" in self.snapshot[pkg_name].keys():
                tag_or_rev = self.snapshot[pkg_name].get("tag", None)
                ref_type = "tag"
            elif "branch" in self.snapshot[pkg_name].keys():
                tag_or_rev = self.snapshot[pkg_name].get("branch", None)
                ref_type = "branch"
            else:
                tag_or_rev = self.snapshot[pkg_name].get("rev", None)
                ref_type = "rev"

            return url, tag_or_rev, ref_type

        pkg = self._distro.release_packages[pkg_name]
        repo = self._distro.repositories[pkg.repository_name].release_repository
        release_tag = get_release_tag(repo, pkg_name)
        return repo.url, release_tag, "tag"

    def check_package(self, pkg_name):
        # If the package is in the additional_packages_snapshot, it is always considered valid
        # even if it is not in the released packages, as it is an additional
        # package specified in rosdistro_additional_recipes.yaml
        if (
            self.additional_packages_snapshot
            and pkg_name in self.additional_packages_snapshot
        ):
            return True
        # the .replace('_', '-') is needed for packages like 'hpp-fcl' that have hypen and not underscore
        # in the rosdistro metadata
        if (
            pkg_name in self._distro.release_packages
            or pkg_name.replace("_", "-") in self._distro.release_packages
        ):
            return self.snapshot is None or (
                pkg_name in self.snapshot or pkg_name.replace("_", "-") in self.snapshot
            )
        elif pkg_name in self.build_packages:
            return True
        else:
            return False

    def get_version(self, pkg_name):
        if self.snapshot and pkg_name in self.snapshot:
            return self.snapshot[pkg_name].get("version", None)

        pkg = self._distro.release_packages[pkg_name]
        repo = self._distro.repositories[pkg.repository_name].release_repository
        return repo.version.split("-")[0]

    def get_release_package(self, pkg_name):
        """Return a parsed catkin_pkg.Package, or None if not available.

        Dispatches between the additional-packages snapshot (which may use
        either pixi.toml or package.xml) and the upstream rosdistro release.
        Conditional deps are evaluated against the current environment.
        """
        if (
            self.additional_packages_snapshot
            and pkg_name in self.additional_packages_snapshot
        ):
            pkg_info = self.additional_packages_snapshot[pkg_name]
            filename, content = self._fetch_additional_manifest(pkg_info)
            pkg = parse_additional_manifest(
                filename,
                content,
                ros_distro=self.distro_name,
                source=f"{pkg_info.get('url', '<unknown>')}@{pkg_info.get('rev') or pkg_info.get('tag') or pkg_info.get('branch')}",
            )
        else:
            xml = self._distro.get_release_package_xml(pkg_name)
            if not xml:
                return None
            pkg = catkin_pkg.package.parse_package_string(xml)

        pkg.evaluate_conditions(os.environ)
        return pkg

    def check_ros1(self):
        return self._distribution_type == "ros1"

    def get_python_version(self):
        return self._python_version

    def get_package_names(self):
        return self._distro.release_packages.keys()

    # Based on https://github.com/ros-infrastructure/rosdistro/blob/fad8d9f647631945847cb18bc1d1f43008d7a282/src/rosdistro/manifest_provider/github.py#L51C1-L69C29
    # Manifest filename defaults to `pixi.toml` (Greenroom convention) but
    # can be overridden via `manifest_file` or the legacy `package_xml_name`.
    def _fetch_additional_manifest(self, pkg_info):
        """Fetch the manifest file for an additional package from GitHub.

        Returns (filename, content) so callers can dispatch on filename to
        pick the right parser.
        """
        raw_url_base = pkg_info.get("url")
        if raw_url_base.endswith(".git"):
            raw_url_base = raw_url_base[:-4]
        if "github.com" not in raw_url_base:
            raise RuntimeError(f"Cannot handle non-GitHub URL: {raw_url_base}")
        owner_repo = raw_url_base.split("github.com/")[-1]
        ref = pkg_info.get("rev") or pkg_info.get("tag") or pkg_info.get("branch")
        manifest_name = pkg_info.get(
            "manifest_file", pkg_info.get("package_xml_name", "pixi.toml")
        )
        additional_folder = pkg_info.get("additional_folder", "")
        path = (
            f"{additional_folder}/{manifest_name}" if additional_folder else manifest_name
        )
        api_url = (
            f"https://api.github.com/repos/{owner_repo}/contents/"
            f"{urllib.parse.quote(path)}"
            f"?ref={urllib.parse.quote(ref, safe='')}"
        )
        if api_url in self._additional_xml_cache:
            return manifest_name, self._additional_xml_cache[api_url]

        try:
            resp = vinca_http.fetch(
                api_url, headers={"Accept": "application/vnd.github.raw"}
            )
            content = resp.text
            self._additional_xml_cache[api_url] = content
            return manifest_name, content
        except Exception as e:
            raise RuntimeError(f"Failed to fetch {manifest_name} from {api_url}: {e}")
