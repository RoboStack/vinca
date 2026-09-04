import os

from vinca.sources import _relative_patch_paths, generate_source, source_reference


class FakeDistro:
    name = "humble"

    def check_package(self, _name):
        return True

    def get_version(self, _name):
        return "1.2.3"

    def get_released_repo(self, _name):
        return "https://example.com/demo.tar.gz", "abc123", "sha256"

    def get_legacy_package_prefix(self):
        return "ros-humble"


def test_source_reference_supports_archives_and_git():
    assert source_reference(url="archive", ref="sum", ref_type="sha256") == {
        "url": "archive",
        "sha256": "sum",
    }
    assert source_reference(url="repo", ref="v1", ref_type="git_tag") == {
        "git": "repo",
        "git_tag": "v1",
    }


def test_generate_source_combines_generic_and_platform_patches(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    generic = tmp_path / "patches" / "demo.patch"
    platform_patch = tmp_path / "patches" / "demo.linux.patch"
    generic.parent.mkdir()
    generic.touch()
    platform_patch.touch()
    config = {
        "_selected_pkgs": ["demo"],
        "_conda_indexes": [],
        "_pkg_additional_info": {},
        "_patches": {
            "ros-humble-demo": {
                "any": [str(generic)],
                "linux": [str(platform_patch)],
            }
        },
        "ros_distro": "humble",
        "package_name_mode": "legacy",
        "skip_built_packages": [],
    }

    sources = generate_source(FakeDistro(), config, "linux-64")

    assert sources["ros-humble-demo"] == {
        "url": "https://example.com/demo.tar.gz",
        "sha256": "abc123",
        "target_directory": "ros-humble-demo/src/work",
        "patches": ["patches/demo.patch", "patches/demo.linux.patch"],
    }


def test_relative_patch_paths_use_posix_separators(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    patch = tmp_path / "patches" / "demo.patch"

    assert _relative_patch_paths([str(patch)]) == ["patches/demo.patch"]


def test_relative_patch_paths_fall_back_to_absolute_without_a_shared_root(monkeypatch):
    def explode(_paths):
        raise ValueError("paths don't have the same drive")

    monkeypatch.setattr(os.path, "commonpath", explode)

    assert _relative_patch_paths(["/elsewhere/demo.patch"]) == ["/elsewhere/demo.patch"]


def test_generate_source_skips_already_built_package():
    config = {
        "_selected_pkgs": ["demo"],
        "_conda_indexes": [],
        "_pkg_additional_info": {},
        "_patches": {},
        "ros_distro": "humble",
        "package_name_mode": "legacy",
        "skip_built_packages": ["ros-humble-demo"],
    }

    assert generate_source(FakeDistro(), config, "linux-64") == {}
