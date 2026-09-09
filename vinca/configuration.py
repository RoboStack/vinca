"""Loading and normalization of ``vinca.yaml`` configuration.

:func:`read_vinca_yaml` is the single place where configuration is read from disk.
Beyond parsing it resolves selectors for the target platform and discovers the
files that live alongside the config: patches, test definitions, dependency
overrides, per-package metadata and rosdistro snapshots.

Derived values are written back into the returned mapping under underscore-prefixed
keys (``_patches``, ``_tests``, ``_conda_indexes``, ``_snapshot`` and friends). Those
keys are internal to vinca and are never read from the user's file.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from ruamel.yaml import YAML

from vinca import config
from vinca.naming import get_package_name_mode
from vinca.resolve import get_conda_index
from vinca.utils import add_package_name_variants
from vinca.v1_selectors import evaluate_selectors

_PATCH_PLATFORMS = ("osx", "linux", "win", "emscripten")


def _load_yaml(path: Path) -> Any:
    yaml = YAML()
    with path.open(encoding="utf-8") as stream:
        return yaml.load(stream)


def _load_selected_yaml(path: Path, target_platform: str) -> Any:
    return evaluate_selectors(_load_yaml(path), target_platform=target_platform)


def _normalize_conda_indexes(indexes: list[str]) -> list[str]:
    """Make local index files absolute while leaving remote URLs untouched."""
    return [
        str(Path(index).absolute()) if Path(index).is_file() else index
        for index in indexes
    ]


def _discover_patches(
    patch_dir: Path, ros_distro: str
) -> dict[str, dict[str, list[str]]]:
    """Group ``<package>[.<platform>].patch`` files by package and target platform.

    ``unix`` is expanded to both ``linux`` and ``osx``; an unrecognized middle
    segment is treated as part of the package name and the patch applies anywhere.
    """
    patches: dict[str, dict[str, list[str]]] = {}
    for path in sorted(patch_dir.glob("*.patch")):
        parts = path.name.split(".")
        package_patches = patches.setdefault(
            parts[0], {"any": [], **{platform: [] for platform in _PATCH_PLATFORMS}}
        )
        destination_platforms = ["any"]
        if len(parts) == 3:
            if parts[1] in _PATCH_PLATFORMS:
                destination_platforms = [parts[1]]
            elif parts[1] == "unix":
                destination_platforms = ["linux", "osx"]
        for platform in destination_platforms:
            package_patches[platform].append(str(path))

    add_package_name_variants(patches, ros_distro)
    return patches


def _discover_tests(
    config_dir: Path, ros_distro: str
) -> tuple[dict[str, Path], dict[str, Path]]:
    """Find per-package test definitions and test data folders next to the config."""
    test_dir = config_dir / "tests"
    tests = {path.name.split(".")[0]: path for path in test_dir.glob("*.yaml")}
    test_folders = {path.name: path for path in test_dir.glob("*") if path.is_dir()}
    add_package_name_variants(tests, ros_distro)
    add_package_name_variants(test_folders, ros_distro)
    return tests, test_folders


def read_snapshot(
    vinca_conf: dict[str, Any],
) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
    """Load the primary and optional additional package snapshots."""
    snapshot_path = vinca_conf.get("rosdistro_snapshot")
    if not snapshot_path:
        return None, None

    snapshot = _load_yaml(Path(snapshot_path)) or {}
    additional_path = vinca_conf.get("rosdistro_additional_recipes")
    additional = _load_yaml(Path(additional_path)) or {} if additional_path else None
    if additional:
        snapshot.update(additional)
    return snapshot, additional


def read_vinca_yaml(filepath: str | Path, target_platform: str) -> dict[str, Any]:
    """Read a vinca configuration and populate its derived internal fields."""
    filepath = Path(filepath)
    config_dir = filepath.parent
    vinca_conf = _load_selected_yaml(filepath, target_platform)
    vinca_conf["package_name_mode"] = get_package_name_mode(vinca_conf).value
    vinca_conf["conda_index"] = _normalize_conda_indexes(vinca_conf["conda_index"])

    patch_dir = Path(vinca_conf["patch_dir"]).absolute()
    vinca_conf["_patch_dir"] = patch_dir
    vinca_conf["_patches"] = _discover_patches(patch_dir, vinca_conf["ros_distro"])
    vinca_conf["_tests"], vinca_conf["_test_folders"] = _discover_tests(
        config_dir, vinca_conf["ros_distro"]
    )

    dependencies_path = patch_dir / "dependencies.yaml"
    if dependencies_path.exists():
        vinca_conf["depmods"] = _load_selected_yaml(dependencies_path, target_platform)
    vinca_conf["depmods"] = vinca_conf.get("depmods") or {}

    config.ros_distro = vinca_conf["ros_distro"]
    config.skip_testing = vinca_conf.get("skip_testing", True)
    config.setup_pixi_version = vinca_conf.get("setup_pixi_version")
    config.pixi_version = vinca_conf.get("pixi_version")
    vinca_conf["_conda_indexes"] = get_conda_index(vinca_conf, str(config_dir))
    vinca_conf["trigger_new_versions"] = vinca_conf.get("trigger_new_versions", False)

    additional_info_path = config_dir / "pkg_additional_info.yaml"
    vinca_conf["_pkg_additional_info"] = (
        _load_selected_yaml(additional_info_path, target_platform)
        if additional_info_path.exists()
        else {}
    )

    snapshot, additional = read_snapshot(vinca_conf)
    vinca_conf["_snapshot"] = snapshot or {}
    vinca_conf["_additional_packages_snapshot"] = additional or {}
    return vinca_conf
