"""Loading and normalization of ``vinca.yaml`` configuration.

:func:`read_vinca_yaml` is the single place where configuration is read from disk.
Beyond parsing it resolves selectors for the target platform and discovers the
files that live alongside the config: patches, test definitions, dependency
overrides, per-package metadata and rosdistro snapshots.

The result is a :class:`~vinca.schemas.VincaConfig`: the validated ``vinca.yaml``
together with what is derived from those files (``patches``, ``tests``,
``conda_indexes``, ``snapshot`` and friends), which are never read from the user's
file.
"""

from __future__ import annotations

import re
import warnings
from pathlib import Path
from typing import Any, Optional, TypeVar

from pydantic import BaseModel
from ruamel.yaml import YAML

from vinca import config
from vinca.resolve import get_conda_index
from vinca.schemas import (
    CondaIndex,
    DepMods,
    PkgAdditionalInfo,
    RecipeTests,
    Snapshot,
    VariantsMode,
    VincaConfig,
    VincaYaml,
    VincaYamlLayer,
    validate,
)
from vinca.utils import add_package_name_variants
from vinca.v1_selectors import evaluate_distro_selectors, evaluate_selectors
from vinca.variants import convert_v0_variant_selectors

_PATCH_PLATFORMS = ("osx", "linux", "win", "emscripten")

_Entry = TypeVar("_Entry", bound=BaseModel)


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


def read_recipe_tests(path: Path, ros_distro: Optional[str]) -> RecipeTests:
    """A package's test definitions, with its ``ros_distro`` selectors resolved."""
    data = YAML(typ="safe").load(path.read_text(encoding="utf-8"))
    return validate(
        RecipeTests,
        evaluate_distro_selectors(data, ros_distro=ros_distro) or {},
        path,
    )


def _load_snapshot(path: Path) -> Snapshot:
    return validate(Snapshot, _load_yaml(path) or {}, path)


def read_snapshot(
    vinca_conf: VincaYaml,
) -> tuple[Optional[Snapshot], Optional[Snapshot]]:
    """Load the primary and optional additional package snapshots."""
    if not vinca_conf.rosdistro_snapshot:
        return None, None

    snapshot = _load_snapshot(Path(vinca_conf.rosdistro_snapshot))
    additional_path = vinca_conf.rosdistro_additional_recipes
    additional = _load_snapshot(Path(additional_path)) if additional_path else None
    if additional:
        snapshot.update(additional)
    return snapshot, additional


def _names(items: Any) -> list[str]:
    """Package names of a (selector-resolved) package list, as written."""
    return [str(item) for item in items or [] if item is not None]


def _apply_package_exclusions(
    vinca_conf: dict[str, Any],
) -> tuple[list[str], list[str]]:
    """Normalize ``packages_exclude`` / ``packages_skip``.

    ``packages_exclude``: the package is not built, and is dropped from the host and
    run dependencies of every other package.

    ``packages_skip``: the package is not built, but packages that depend on it keep
    the dependency, e.g. on a build that is already published.

    Both also remove the package from ``packages_select_by_deps``. Selectors
    (``- if: ... then: ...``) have already been resolved for the target platform.

    Returns what the dependency traversal ignores, and what is dropped from
    dependencies.
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
    dropped = {name.replace("-", "_") for name in exclude + skip}
    vinca_conf["packages_select_by_deps"] = [
        name
        for name in _names(vinca_conf.get("packages_select_by_deps"))
        if name.replace("-", "_") not in dropped
    ]
    return (
        list(dict.fromkeys(exclude + skip + legacy_skip)),
        list(dict.fromkeys(exclude + legacy_remove)),
    )


# paths in a configuration, relative to the file that sets them
_PATH_KEYS = ("patch_dir", "rosdistro_snapshot", "rosdistro_additional_recipes")

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


def _merge_per_package(
    base: dict[str, _Entry], own: dict[str, _Entry]
) -> dict[str, _Entry]:
    """Per-package settings: the keys set in own's entry of a package win over the
    base's."""
    merged = dict(base)
    for package, entry in own.items():
        if package in merged:
            merged[package] = merged[package].model_copy(
                update={key: getattr(entry, key) for key in entry.model_fields_set}
            )
        else:
            merged[package] = entry
    return merged


def _load_layers(
    filepath: Path, target_platform: str, seen: tuple[Path, ...] = ()
) -> list[tuple[Path, dict[str, Any]]]:
    """The configuration and the ones it ``extends`` (resolved relative to the
    file), base first, each as (file, selector-resolved content)."""
    filepath = filepath.resolve()
    if filepath in seen:
        chain = " -> ".join(str(p) for p in (*seen, filepath))
        raise ValueError(f"vinca.yaml extends itself: {chain}")
    conf = _load_selected_yaml(filepath, target_platform) or {}
    for key in _PATH_KEYS:
        if conf.get(key):
            conf[key] = str((filepath.parent / conf[key]).resolve())
    base = conf.pop("extends", None)
    layers = (
        _load_layers(filepath.parent / base, target_platform, (*seen, filepath))
        if base
        else []
    )
    return [*layers, (filepath, conf)]


def read_vinca_yaml(filepath: str | Path, target_platform: str) -> VincaConfig:
    """Read a vinca configuration and populate its derived internal fields."""
    filepath = Path(filepath)
    config_dir = filepath.parent
    layers = _load_layers(filepath, target_platform)
    merged: dict[str, Any] = {}
    for _, layer in layers:
        merged = _merge_configs(merged, layer)
    skip_by_deps, remove_from_deps = _apply_package_exclusions(merged)
    settings = validate(VincaYaml, merged, filepath)
    ros_distro = settings.ros_distro

    # Files of each layer: patches and dependencies.yaml (patch_dir), tests,
    # pkg_additional_info.yaml and conda_index files, relative to that layer's file.
    patches: dict[str, dict[str, list[str]]] = {}
    tests: dict[str, Path] = {}
    test_folders: dict[str, Path] = {}
    depmods: dict[str, DepMods] = {}
    additional_info: dict[str, PkgAdditionalInfo] = {}
    conda_indexes: list[CondaIndex] = []
    patch_dir: Path | None = None
    for layer_file, layer_content in layers:
        layer_dir = layer_file.parent
        layer = validate(VincaYamlLayer, layer_content, layer_file)
        if layer.patch_dir:
            layer_patch_dir = Path(layer.patch_dir)
            patches.update(_discover_patches(layer_patch_dir, ros_distro))
            dependencies_path = layer_patch_dir / "dependencies.yaml"
            if dependencies_path.exists():
                depmods = _merge_per_package(
                    depmods,
                    validate(
                        dict[str, DepMods],
                        _load_selected_yaml(dependencies_path, target_platform) or {},
                        dependencies_path,
                    ),
                )
            patch_dir = layer_patch_dir
        layer_tests, layer_test_folders = _discover_tests(layer_dir, ros_distro)
        tests.update(layer_tests)
        test_folders.update(layer_test_folders)
        additional_info_path = layer_dir / "pkg_additional_info.yaml"
        if additional_info_path.exists():
            additional_info = _merge_per_package(
                additional_info,
                validate(
                    dict[str, PkgAdditionalInfo],
                    _load_selected_yaml(additional_info_path, target_platform) or {},
                    additional_info_path,
                ),
            )
        if layer.conda_index:
            layer_indexes = [
                str(layer_dir / i) if (layer_dir / i).is_file() else i
                for i in layer.conda_index
            ]
            # the extending configuration's mappings are looked up first
            conda_indexes = (
                get_conda_index(layer_indexes, str(layer_dir)) + conda_indexes
            )

    config.ros_distro = ros_distro
    config.skip_testing = settings.skip_testing
    config.setup_pixi_version = settings.setup_pixi_version
    config.pixi_version = settings.pixi_version

    variant_config: dict[str, Any] = {}
    if settings.variants_mode is VariantsMode.LOCAL:
        variant_config_path = config_dir / "conda_build_config.yaml"
        if not variant_config_path.is_file():
            raise FileNotFoundError(
                "variants_mode 'local' requires conda_build_config.yaml next to vinca.yaml"
            )
        variant_config = convert_v0_variant_selectors(
            _load_yaml(variant_config_path) or {}
        )

    snapshot, additional = read_snapshot(settings)
    fields = dict(settings)
    fields.update(
        conda_index=_normalize_conda_indexes(settings.conda_index),
        depmods=depmods or settings.depmods,
    )
    return VincaConfig(
        **fields,
        skip_by_deps=skip_by_deps,
        remove_from_deps=remove_from_deps,
        patch_dir_path=patch_dir,
        patches=patches,
        tests=tests,
        test_folders=test_folders,
        conda_indexes=conda_indexes,
        pkg_additional_info=additional_info,
        variant_config=variant_config,
        snapshot=snapshot or {},
        additional_packages_snapshot=additional or {},
    )
