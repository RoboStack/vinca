import os

from rosdistro import get_cached_distribution, get_index, get_index_url
from rosdistro.dependency_walker import DependencyWalker
from rosdistro.manifest_provider import get_release_tag


class Distro(object):

    def __init__(self, distro_name):
        index = get_index(get_index_url())
        self._distro = get_cached_distribution(index, distro_name)
        self._walker = DependencyWalker(
            self._distro,
            evaluate_condition_context=os.environ)

    def get_depends(self, pkg):
        dependencies = set()
        dependencies |= self._walker.get_recursive_depends(
            pkg,
            ['buildtool', 'buildtool_export', 'build', 'build_export',
             'run', 'test', 'exec'],
            ros_packages_only=True)
        return dependencies

    def get_released_repo(self, pkg_name):
        pkg = self._distro.release_packages[pkg_name]
        repo = self._distro.repositories[pkg.repository_name].release_repository
        release_tag = get_release_tag(repo, pkg_name)
        return repo.url, release_tag

    def check_package(self, pkg_name):
        if pkg_name in self._distro.release_packages:
            return True
        else:
            return False

    def get_version(self, pkg_name):
        pkg = self._distro.release_packages[pkg_name]
        repo = self._distro.repositories[pkg.repository_name].release_repository
        return repo.version

    def get_release_package_xml(self, pkg_name):
        return self._distro.get_release_package_xml(pkg_name)
