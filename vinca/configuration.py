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

import re
import warnings
from pathlib import Path
from typing import Any

from ruamel.yaml import YAML

from vinca import config
from vinca.naming import get_package_name_mode
from vinca.resolve import get_conda_index
from vinca.utils import add_package_name_variants
from vinca.v1_selectors import evaluate_selectors
from vinca.variants import (
    VariantsMode,
    convert_v0_variant_selectors,
    get_variants_mode,
)

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


def _names(items: Any) -> list[str]:
    """Package names of a (selector-resolved) package list, as written."""
    return [str(item) for item in items or [] if item is not None]


def _apply_package_exclusions(vinca_conf: dict[str, Any]) -> None:
    """Normalize ``packages_exclude`` / ``packages_skip``.

    ``packages_exclude``: the package is not built, and is dropped from the host and
    run dependencies of every other package.

    ``packages_skip``: the package is not built, but packages that depend on it keep
    the dependency, e.g. on a build that is already published.

    Both also remove the package from ``packages_select_by_deps``. Selectors
    (``- if: ... then: ...``) have already been resolved for the target platform.
    """
    exclude = list(dict.fromkeys(_names(vinca_conf.get("packages_exclude"))))
    skip = list(dict.fromkeys(_names(vinca_conf.get("packages_skip"))))
    # The keys these replace still work as before, so that existing configurations
    # keep working with a newer vinca.
    legacy_skip = _names(vinca_conf.pop("packages_skip_by_deps", None))
    legacy_remove = _names(vinca_conf.pop("packages_remove_from_deps", None))
    if legacy_skip or legacy_remove:
        warnings.warn(
            "packages_skip_by_deps and packages_remove_from_deps are deprecated: list a "
            "package in packages_exclude if it was in packages_remove_from_deps (not "
            "built, dropped from other packages' dependencies), or in packages_skip if it "
            "was only in packages_skip_by_deps (not built, dependents keep the dependency)",
            FutureWarning,
            stacklevel=2,
        )
    vinca_conf["packages_exclude"] = exclude
    vinca_conf["packages_skip"] = skip
    # what the dependency traversal ignores, and what is dropped from dependencies
    vinca_conf["_skip_by_deps"] = list(dict.fromkeys(exclude + skip + legacy_skip))
    vinca_conf["_remove_from_deps"] = list(dict.fromkeys(exclude + legacy_remove))
    dropped = {name.replace("-", "_") for name in exclude + skip}
    vinca_conf["packages_select_by_deps"] = [
        name
        for name in _names(vinca_conf.get("packages_select_by_deps"))
        if name.replace("-", "_") not in dropped
    ]


# package lists that a configuration adds to the ones of the configuration it extends
_LAYERED_LISTS = ("packages_select_by_deps", "packages_exclude", "packages_skip")


def _constraint_name(spec: Any) -> str:
    return re.split(r"[\s=<>!~]", str(spec).strip(), maxsplit=1)[0]


def _merge_mutex(base: dict[str, Any], own: dict[str, Any]) -> dict[str, Any]:
    """``mutex_package``: own keys win; own ``run_constraints`` replace the base's
    constraint on the same package and add the others."""
    merged = {**base, **own}
    mine = list(own.get("run_constraints") or [])
    names = {_constraint_name(c) for c in mine}
    merged["run_constraints"] = [
        c for c in base.get("run_constraints") or [] if _constraint_name(c) not in names
    ] + mine
    return merged


def _merge_configs(base: dict[str, Any], own: dict[str, Any]) -> dict[str, Any]:
    """A configuration on top of the one it extends: package lists are combined,
    ``mutex_package`` is merged, any other key of ``own`` replaces the base's."""
    merged = dict(base)
    for key, value in own.items():
        if key in _LAYERED_LISTS:
            merged[key] = list(dict.fromkeys(_names(base.get(key)) + _names(value)))
        elif key == "mutex_package" and isinstance(base.get(key), dict):
            merged[key] = _merge_mutex(base[key], value or {})
        else:
            merged[key] = value
    return merged


def _merge_per_package(base: dict[str, Any], own: dict[str, Any]) -> dict[str, Any]:
    """Per-package settings: own keys of a package's entry win over the base's."""
    merged = {k: (dict(v) if isinstance(v, dict) else v) for k, v in base.items()}
    for package, entry in own.items():
        if isinstance(entry, dict) and isinstance(merged.get(package), dict):
            merged[package] = {**merged[package], **entry}
        else:
            merged[package] = entry
    return merged


def _load_layers(
    filepath: Path, target_platform: str, seen: tuple[Path, ...] = ()
) -> list[tuple[Path, dict[str, Any]]]:
    """The configuration and the ones it ``extends`` (resolved relative to the
    file), base first, each as (directory, selector-resolved content)."""
    filepath = filepath.resolve()
    if filepath in seen:
        chain = " -> ".join(str(p) for p in (*seen, filepath))
        raise ValueError(f"vinca.yaml extends itself: {chain}")
    conf = _load_selected_yaml(filepath, target_platform) or {}
    base = conf.pop("extends", None)
    layers = (
        _load_layers(filepath.parent / base, target_platform, (*seen, filepath))
        if base
        else []
    )
    return [*layers, (filepath.parent, conf)]


def read_vinca_yaml(filepath: str | Path, target_platform: str) -> dict[str, Any]:
    """Read a vinca configuration and populate its derived internal fields."""
    filepath = Path(filepath)
    config_dir = filepath.parent
    layers = _load_layers(filepath, target_platform)
    vinca_conf: dict[str, Any] = {}
    for _, layer in layers:
        vinca_conf = _merge_configs(vinca_conf, layer)
    _apply_package_exclusions(vinca_conf)
    vinca_conf["package_name_mode"] = get_package_name_mode(vinca_conf).value
    vinca_conf["variants_mode"] = get_variants_mode(vinca_conf).value

    # Files next to each layer: patches and dependencies.yaml (patch_dir), tests,
    # pkg_additional_info.yaml and conda_index files. Paths of a base configuration
    # are relative to its own file; the extending configuration (the root) keeps
    # resolving patch_dir against the working directory, as without extends.
    patches: dict[str, Any] = {}
    tests: dict[str, Path] = {}
    test_folders: dict[str, Path] = {}
    depmods: dict[str, Any] = {}
    additional_info: dict[str, Any] = {}
    conda_indexes: list[Any] = []
    patch_dir: Path | None = None
    for layer_dir, layer in layers:
        is_root = layer_dir == config_dir.resolve()
        if layer.get("patch_dir"):
            layer_patch_dir = (
                Path(layer["patch_dir"]).absolute()
                if is_root
                else (layer_dir / layer["patch_dir"]).resolve()
            )
            patches.update(_discover_patches(layer_patch_dir, vinca_conf["ros_distro"]))
            dependencies_path = layer_patch_dir / "dependencies.yaml"
            if dependencies_path.exists():
                depmods = _merge_per_package(
                    depmods,
                    _load_selected_yaml(dependencies_path, target_platform) or {},
                )
            patch_dir = layer_patch_dir
        layer_tests, layer_test_folders = _discover_tests(
            layer_dir, vinca_conf["ros_distro"]
        )
        tests.update(layer_tests)
        test_folders.update(layer_test_folders)
        additional_info_path = layer_dir / "pkg_additional_info.yaml"
        if additional_info_path.exists():
            additional_info = _merge_per_package(
                additional_info,
                _load_selected_yaml(additional_info_path, target_platform) or {},
            )
        if layer.get("conda_index"):
            layer_indexes = (
                _normalize_conda_indexes(layer["conda_index"])
                if is_root
                else [
                    str(layer_dir / i) if (layer_dir / i).is_file() else i
                    for i in layer["conda_index"]
                ]
            )
            # the extending configuration's mappings are looked up first
            conda_indexes = (
                get_conda_index({"conda_index": layer_indexes}, str(layer_dir))
                + conda_indexes
            )
    vinca_conf["conda_index"] = _normalize_conda_indexes(
        vinca_conf.get("conda_index") or []
    )
    vinca_conf["_patch_dir"] = patch_dir or Path(vinca_conf["patch_dir"]).absolute()
    vinca_conf["_patches"] = patches
    vinca_conf["_tests"], vinca_conf["_test_folders"] = tests, test_folders
    vinca_conf["depmods"] = depmods or vinca_conf.get("depmods") or {}

    config.ros_distro = vinca_conf["ros_distro"]
    config.skip_testing = vinca_conf.get("skip_testing", True)
    config.setup_pixi_version = vinca_conf.get("setup_pixi_version")
    config.pixi_version = vinca_conf.get("pixi_version")
    vinca_conf["_conda_indexes"] = conda_indexes
    vinca_conf["trigger_new_versions"] = vinca_conf.get("trigger_new_versions", False)
    vinca_conf["_pkg_additional_info"] = additional_info

    vinca_conf["_variant_config"] = {}
    if get_variants_mode(vinca_conf) is VariantsMode.LOCAL:
        variant_config_path = config_dir / "conda_build_config.yaml"
        if not variant_config_path.is_file():
            raise FileNotFoundError(
                "variants_mode 'local' requires conda_build_config.yaml next to vinca.yaml"
            )
        vinca_conf["_variant_config"] = convert_v0_variant_selectors(
            _load_yaml(variant_config_path) or {}
        )

    snapshot, additional = read_snapshot(vinca_conf)
    vinca_conf["_snapshot"] = snapshot or {}
    vinca_conf["_additional_packages_snapshot"] = additional or {}
    return vinca_conf
