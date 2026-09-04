import io
import os
import posixpath
import tarfile
import urllib.parse
import zipfile
from concurrent.futures import ThreadPoolExecutor
from typing import Iterable, Optional

import catkin_pkg.package
import requests
from rosdistro import get_cached_distribution, get_index, get_index_url
from rosdistro.dependency_walker import DependencyWalker
from rosdistro.manifest_provider import get_release_tag

# Source archive suffixes an additional recipe may point at instead of a git repository.
# Such a package is fetched by URL (checksummed with sha256) rather than cloned.
# Zstandard is left out on purpose: tarfile only reads it from Python 3.14 on, while
# vinca still supports older interpreters, so accepting it here would promise a format
# that fails to open on most of them.
ARCHIVE_SUFFIXES = (
    ".zip",
    ".tar",
    ".tar.gz",
    ".tgz",
    ".tar.bz2",
    ".tbz2",
    ".tar.xz",
    ".txz",
)


def is_archive_url(url):
    """Return True when url points at a source archive rather than a git repository."""
    if not url:
        return False
    path = urllib.parse.urlparse(url).path.lower()
    return path.endswith(ARCHIVE_SUFFIXES)


def is_bloom_release_repository_url(url):
    """Return True when url names a Bloom-generated release repository."""
    path = urllib.parse.urlparse(url).path.rstrip("/")
    repository_name = posixpath.basename(path).removesuffix(".git").lower()
    return repository_name.endswith(("-release", "_release"))


def _strip_git_suffix(url):
    """Return a repository URL without its optional ``.git`` suffix."""
    return url[:-4] if url.lower().endswith(".git") else url


def _normalize_member(name):
    """Return an archive member name without its './' prefix and trailing slash."""
    name = name.strip("/")
    while name.startswith("./"):
        name = name[2:]
    return "" if name == "." else name


def _strip_common_root(entries):
    """Return the single top-level directory shared by entries, or an empty string.

    Entries are (name, is_dir) pairs. Build tools unpack an archive whose contents all
    live under one directory by dropping that directory, so member lookups do the same.
    """
    names = [(_normalize_member(name), is_dir) for name, is_dir in entries]
    roots = {name.split("/", 1)[0] for name, _ in names if name}
    if len(roots) != 1:
        return ""
    root = roots.pop()
    # A lone top-level file, rather than a directory, is not a root to strip.
    if any(name == root and not is_dir for name, is_dir in names):
        return ""
    return root


def _resolve_member(*, entries, member):
    """Return the name entries use for member, resolved against the stripped root.

    Names are compared normalized, so an archive listing its members as './pkg/...'
    resolves like one listing them as 'pkg/...'.
    """
    wanted = posixpath.join(_strip_common_root(entries), member)
    for name, is_dir in entries:
        if not is_dir and _normalize_member(name) == wanted:
            return name
    raise KeyError(member)


def _read_archive_member(*, payload, url, member):
    """Return the text of member inside the archive payload, stripping its common root."""
    member = _normalize_member(member)
    if zipfile.is_zipfile(io.BytesIO(payload)):
        with zipfile.ZipFile(io.BytesIO(payload)) as archive:
            entries = [(info.filename, info.is_dir()) for info in archive.infolist()]
            return archive.read(_resolve_member(entries=entries, member=member)).decode(
                "utf-8"
            )
    try:
        archive = tarfile.open(fileobj=io.BytesIO(payload), mode="r:*")
    except tarfile.TarError as error:
        raise RuntimeError(f"Unsupported archive format: {url}") from error
    with archive:
        entries = [(m.name, m.isdir()) for m in archive.getmembers()]
        extracted = archive.extractfile(_resolve_member(entries=entries, member=member))
        if extracted is None:
            raise KeyError(member)
        return extracted.read().decode("utf-8")


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
        self._last_archive = None
        self._depends_cache = {}
        self._direct_depends_cache = {}

        os.environ["ROS_VERSION"] = "1" if self.check_ros1() else "2"

    @property
    def name(self):
        return self.distro_name

    def add_packages(self, packages):
        self.build_packages = set(packages)

    def get_depends(
        self, pkg: str, ignore_pkgs: Optional[Iterable[str]] = None
    ) -> set[str]:
        cache_key = (pkg, tuple(sorted(ignore_pkgs)) if ignore_pkgs else None)
        if cache_key in self._depends_cache:
            return set(self._depends_cache[cache_key])

        if not self.check_package(pkg):
            print(f"{pkg} not in available packages anymore")
            return set()

        ignore_pkgs = set(ignore_pkgs or ())

        if self.snapshot:
            dependencies = self._get_snapshot_recursive_depends(pkg, ignore_pkgs)
            self._depends_cache[cache_key] = set(dependencies)
            return dependencies

        dependencies = set()
        visited = {pkg}
        packages_to_check = {pkg}
        while packages_to_check:
            current = packages_to_check.pop()
            if current in ignore_pkgs:
                continue
            direct = self._get_direct_depends(current) - ignore_pkgs
            new_dependencies = direct - visited
            visited.update(new_dependencies)
            dependencies.update(new_dependencies)
            packages_to_check.update(
                dependency
                for dependency in new_dependencies
                if self.check_package(dependency)
            )

        self._depends_cache[cache_key] = set(dependencies)
        return dependencies

    def _get_direct_depends(self, pkg: str) -> set[str]:
        """Return direct dependencies, caching package metadata across root walks."""

        if pkg in self._direct_depends_cache:
            return set(self._direct_depends_cache[pkg])

        # if pkg comes from additional_packages_snapshot, extract from its package.xml
        if (
            self.additional_packages_snapshot
            and pkg in self.additional_packages_snapshot
        ):
            pkg_info = self.additional_packages_snapshot[pkg]
            xml_str = self.get_package_xml_for_additional_package(pkg_info)
            # parse XML
            import xml.etree.ElementTree as ET

            root = ET.fromstring(xml_str)
            # collect direct dependencies tags from package.xml
            dep_tags = [
                "depend",
                "build_depend",
                "buildtool_depend",
                "buildtool_export_depend",
                "exec_depend",
                "run_depend",
                "test_depend",
                "build_export_depend",
            ]
            direct = set()
            for tag in dep_tags:
                for elem in root.findall(f".//{tag}"):
                    if elem.text:
                        name = elem.text.strip()
                        direct.add(name)
            self._direct_depends_cache[pkg] = set(direct)
            return direct

        # Cache the union before walking the graph. DependencyWalker otherwise
        # recomputes and deep-copies these fields for every configured root package.
        direct = set()
        for dependency_type in (
            "buildtool",
            "buildtool_export",
            "build",
            "build_export",
            "run",
            "test",
            "exec",
        ):
            direct.update(
                self._walker.get_depends(pkg, dependency_type, ros_packages_only=True)
            )
        self._direct_depends_cache[pkg] = set(direct)
        return direct

    def _get_snapshot_recursive_depends(
        self, pkg: str, ignore_pkgs: Optional[Iterable[str]] = None
    ) -> set[str]:
        """Return ROS dependencies using only package manifests pinned by the snapshot."""
        dependencies: set[str] = set()
        ignored = set(ignore_pkgs or [])
        packages_to_check = {pkg}
        checked_packages = set()
        dependency_attributes = (
            "buildtool_depends",
            "buildtool_export_depends",
            "build_depends",
            "build_export_depends",
            "run_depends",
            "test_depends",
            "exec_depends",
        )

        while packages_to_check:
            package_name = sorted(packages_to_check)[0]
            packages_to_check.remove(package_name)
            if package_name in ignored or package_name in checked_packages:
                continue
            checked_packages.add(package_name)

            package_xml = self.get_release_package_xml(package_name)
            package = catkin_pkg.package.parse_package_string(package_xml)
            package.evaluate_conditions(os.environ)
            direct_dependencies: set[str] = {
                dependency.name
                for attribute in dependency_attributes
                for dependency in getattr(package, attribute)
                if dependency.evaluated_condition is not False
                and dependency.name not in ignored
                and self.check_package(dependency.name)
            }
            new_dependencies = direct_dependencies - dependencies
            dependencies |= new_dependencies
            packages_to_check |= new_dependencies - checked_packages

        return dependencies

    def _get_snapshot_package_info(self, pkg_name):
        if not self.snapshot:
            return None
        for name in (pkg_name, pkg_name.replace("_", "-")):
            if name in self.snapshot:
                return self.snapshot[name]
        return None

    def get_released_repo(self, pkg_name):
        pkg_info = self._get_snapshot_package_info(pkg_name)
        if pkg_info is not None:
            # In the case of snapshot, for rosdistro_additional_recipes
            # we also support a 'rev' field, so depending on what is available
            # we return either the tag or the rev, and the third argument is either 'rev' or 'tag'
            url = pkg_info.get("url", None)
            # An archive URL is not a git repository: the reference is its sha256 checksum.
            if is_archive_url(url):
                sha256 = pkg_info.get("sha256", None)
                if not sha256:
                    raise RuntimeError(
                        f"The archive source of '{pkg_name}' has no sha256 checksum: {url}"
                    )
                return url, sha256, "sha256"
            if "tag" in pkg_info:
                tag_or_rev = pkg_info.get("tag", None)
                ref_type = "tag"
            else:
                tag_or_rev = pkg_info.get("rev", None)
                ref_type = "rev"

            return url, tag_or_rev, ref_type

        pkg = self._distro.release_packages[pkg_name]
        repo = self._distro.repositories[pkg.repository_name].release_repository
        release_tag = get_release_tag(repo, pkg_name)
        return repo.url, release_tag, "tag"

    def get_repository_url(self, pkg_name, package_urls=()):
        """Return the best declared upstream repository for a package."""
        pkg_info = self._get_snapshot_package_info(pkg_name)
        if pkg_info is not None:
            if repository := pkg_info.get("repository"):
                return _strip_git_suffix(repository)

        additional_info = (self.additional_packages_snapshot or {}).get(pkg_name)
        if additional_info is not None:
            if repository := additional_info.get("repository"):
                return _strip_git_suffix(repository)
        else:
            package = self._distro.release_packages.get(pkg_name)
            if package is not None:
                repository = self._distro.repositories[package.repository_name]
                source_repository = repository.source_repository
                if (
                    source_repository is not None
                    and source_repository.url
                    and not is_bloom_release_repository_url(source_repository.url)
                ):
                    return _strip_git_suffix(source_repository.url)

        for package_url in package_urls:
            if (
                package_url.type == "repository"
                and package_url.url
                and not is_bloom_release_repository_url(package_url.url)
            ):
                return _strip_git_suffix(package_url.url)
        return None

    def check_package(self, pkg_name):
        # If the package is in the additional_packages_snapshot, it is always considered valid
        # even if it is not in the released packages, as it is an additional
        # package specified in rosdistro_additional_recipes.yaml
        if (
            self.additional_packages_snapshot
            and pkg_name in self.additional_packages_snapshot
        ):
            return True
        if self.snapshot:
            return (
                self._get_snapshot_package_info(pkg_name) is not None
                or pkg_name in self.build_packages
            )
        # the .replace('_', '-') is needed for packages like 'hpp-fcl' that have a hyphen and not underscore
        # in the rosdistro metadata
        if (
            pkg_name in self._distro.release_packages
            or pkg_name.replace("_", "-") in self._distro.release_packages
        ):
            return True
        elif pkg_name in self.build_packages:
            return True
        else:
            return False

    def get_version(self, pkg_name):
        pkg_info = self._get_snapshot_package_info(pkg_name)
        if pkg_info is not None:
            return pkg_info.get("version", None)

        pkg = self._distro.release_packages[pkg_name]
        repo = self._distro.repositories[pkg.repository_name].release_repository
        return repo.version.split("-")[0]

    def live_cache_matches_snapshot(self, pkg_name, snapshot_entry):
        """Return whether rosdistro's cached manifest is the pinned snapshot source.

        A snapshot pins the release repository URL, the package-specific release
        tag, and the release version.  Only an exact match may reuse rosdistro's
        local ``DistributionCache``; otherwise the manifest must be read from the
        immutable snapshot source.
        """

        for live_name in (pkg_name, pkg_name.replace("_", "-")):
            try:
                package = self._distro.release_packages[live_name]
                repository = self._distro.repositories[package.repository_name]
                release_repository = repository.release_repository
                return (
                    release_repository.url == snapshot_entry.get("url")
                    and get_release_tag(release_repository, live_name)
                    == snapshot_entry.get("tag")
                    and release_repository.version.split("-")[0]
                    == snapshot_entry.get("version")
                )
            except (AttributeError, KeyError, TypeError):
                continue
        return False

    def get_release_package_xml(self, pkg_name):
        pkg_info = self._get_snapshot_package_info(pkg_name)
        if (
            pkg_info is None
            and self.additional_packages_snapshot
            and pkg_name in self.additional_packages_snapshot
        ):
            pkg_info = self.additional_packages_snapshot[pkg_name]
        if pkg_info is not None:
            if self.live_cache_matches_snapshot(pkg_name, pkg_info):
                return self._distro.get_release_package_xml(pkg_name)
            return self.get_package_xml_for_additional_package(pkg_info)
        return self._distro.get_release_package_xml(pkg_name)

    def check_ros1(self):
        return self._distribution_type == "ros1"

    def get_ros_version(self):
        """Get ROS version number (1 or 2)"""
        return "1" if self.check_ros1() else "2"

    def get_package_prefix(self):
        """Get the package name prefix (ros for ROS1, ros2 for ROS2)."""
        return "ros" if self.check_ros1() else "ros2"

    def get_legacy_package_prefix(self):
        """Get the legacy distro-qualified package name prefix."""
        return f"ros-{self.name}"

    def get_python_version(self):
        return self._python_version

    def get_package_names(self):
        if self.snapshot:
            return self.snapshot.keys()
        return self._distro.release_packages.keys()

    def get_package_xml_for_additional_package(self, pkg_info):
        raw_url_base = pkg_info.get("url")
        # An archive has no raw-file endpoint, so read package.xml out of the archive itself.
        if is_archive_url(raw_url_base):
            return self._package_xml_from_archive_or_cached(pkg_info)
        if "github.com" in raw_url_base:
            raw_url = self._construct_raw_url_github(pkg_info)
            return self._download_raw_pkg_xml_or_cached(url=raw_url)
        if "gitlab.com" in raw_url_base:
            raw_url = self._construct_raw_url_gitlab(pkg_info)
            return self._download_raw_pkg_xml_or_cached(url=raw_url)
        raise RuntimeError(f"Cannot handle unknown repository hoster: {raw_url_base}")

    def prefetch_additional_package_xml(self, max_workers: int = 12) -> None:
        """Fetch immutable raw manifests concurrently for dependency discovery."""

        package_infos = [
            package_info
            for package_info in (self.additional_packages_snapshot or {}).values()
            if not is_archive_url(package_info.get("url"))
        ]
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            list(
                executor.map(self.get_package_xml_for_additional_package, package_infos)
            )

    def _get_auth_headers(self, url):
        """Token header for the hosters vinca knows, empty otherwise.

        Any other credential (a private artifact server such as Artifactory, a proxy,
        a custom CA) is resolved by requests from the standard environment, so vinca
        does not need to know about it.
        """
        host = urllib.parse.urlparse(url).netloc
        for env_var, domains in (
            ("GITHUB_TOKEN", ("github.com", "githubusercontent.com")),
            ("GITLAB_TOKEN", ("gitlab.com",)),
        ):
            token = os.environ.get(env_var)
            if token and any(d in host for d in domains):
                return {"Authorization": f"token {token}"}
        return {}

    def _get(self, url):
        """Fetch url, letting requests apply the ambient credentials and proxy settings.

        requests also drops the Authorization header when a redirect leaves the original
        host, which matters because an Artifactory server usually redirects a download
        to a presigned URL on a separate storage host.
        """
        response = requests.get(url, headers=self._get_auth_headers(url), timeout=60)
        response.raise_for_status()
        return response

    def _download_raw_pkg_xml_or_cached(self, url):
        if url in self._additional_xml_cache:
            return self._additional_xml_cache[url]
        try:
            xml_content = self._get(url).text
        except Exception as error:
            raise RuntimeError(
                f"Failed to fetch package.xml from {url}: {error}"
            ) from error
        self._additional_xml_cache[url] = xml_content
        return xml_content

    def _package_xml_from_archive_or_cached(self, pkg_info):
        """Read package.xml out of a source archive referenced by an additional recipe.

        The member is looked up under the archive's additional_folder, after dropping a
        single common top-level directory. That mirrors how the build tool unpacks the
        archive, so one additional_folder value describes both the recipe and this lookup.
        """
        url = pkg_info.get("url")
        xml_name = pkg_info.get("package_xml_name", "package.xml")
        member = posixpath.join(pkg_info.get("additional_folder", ""), xml_name)
        cache_key = f"{url}#{member}"
        if cache_key in self._additional_xml_cache:
            return self._additional_xml_cache[cache_key]

        payload = self._download_archive_or_cached(url)
        try:
            xml_content = _read_archive_member(payload=payload, url=url, member=member)
        except KeyError as error:
            raise RuntimeError(
                f"Could not find '{member}' inside the archive {url}"
            ) from error
        except Exception as error:
            raise RuntimeError(
                f"Failed to read '{member}' from the archive {url}: {error}"
            ) from error
        self._additional_xml_cache[cache_key] = xml_content
        return xml_content

    def _download_archive_or_cached(self, url):
        """Download the archive at url, keeping only the most recent payload in memory.

        Several packages can share one archive, so reusing the last download avoids
        fetching it again without holding every archive of the run in memory.
        """
        if self._last_archive and self._last_archive[0] == url:
            return self._last_archive[1]
        try:
            payload = self._get(url).content
        except Exception as error:
            raise RuntimeError(
                f"Failed to download the archive {url}: {error}"
            ) from error
        self._last_archive = (url, payload)
        return payload

    # Based on https://github.com/ros-infrastructure/rosdistro/blob/fad8d9f647631945847cb18bc1d1f43008d7a282/src/rosdistro/manifest_provider/github.py#L51C1-L69C29
    # But with the option to specify the name of the package.xml file in case the repo uses a non-standard name
    def _construct_raw_url_github(self, pkg_info):
        # Build raw GitHub URL for package.xml
        raw_url_base = pkg_info.get("url")
        if raw_url_base.endswith(".git"):
            raw_url_base = raw_url_base[:-4]
        if "github.com" not in raw_url_base:
            raise RuntimeError(f"Cannot handle non-GitHub URL: {raw_url_base}")
        # Extract owner/repo
        owner_repo = raw_url_base.split("github.com/")[-1]
        # Use rev if available, otherwise fallback to tag
        ref = pkg_info.get("rev") or pkg_info.get("tag")
        xml_name = pkg_info.get("package_xml_name", "package.xml")
        additional_folder = pkg_info.get("additional_folder", "")
        if additional_folder != "":
            additional_folder = additional_folder + "/"
        raw_url = f"https://raw.githubusercontent.com/{owner_repo}/{ref}/{additional_folder}{xml_name}"
        return raw_url

    # format (checked against GitLab 19.x): https://gitlab.com/<NAMESPACE>/-/raw/<REV>/<PATH>
    def _construct_raw_url_gitlab(self, pkg_info):
        raw_url_base = pkg_info.get("url")
        if raw_url_base.endswith(".git"):
            raw_url_base = raw_url_base[:-4]
        if "gitlab.com" not in raw_url_base:
            raise RuntimeError(f"Cannot handle non-GitLab URL: {raw_url_base}")
        # Use rev if available, otherwise fallback to tag
        ref = pkg_info.get("rev") or pkg_info.get("tag")
        xml_name = pkg_info.get("package_xml_name", "package.xml")
        additional_folder = pkg_info.get("additional_folder", "")
        if additional_folder != "":
            additional_folder = additional_folder + "/"
        raw_url = f"{raw_url_base}/-/raw/{ref}/{additional_folder}{xml_name}"
        return raw_url
