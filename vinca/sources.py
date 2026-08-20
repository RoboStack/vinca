"""Source sections for generated rattler-build recipes."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from vinca.mutex import generate_mutex_package_recipe, should_skip_mutex_package
from vinca.resolve import resolve_pkgname
from vinca.utils import is_dummy_metapackage


def source_reference(*, url: str, ref: str, ref_type: str) -> dict[str, str]:
    """Create source keys for either a git reference or a checksummed archive."""
    if ref_type == "sha256":
        return {"url": url, "sha256": ref}
    return {"git": url, ref_type: ref}


def _is_built(package_name: str, version: str, vinca_conf: dict[str, Any]) -> bool:
    skipped = vinca_conf.get("skip_built_packages", [])
    key = (
        (package_name, version)
        if vinca_conf.get("trigger_new_versions")
        else package_name
    )
    return key in skipped


def _package_patches(
    package_name: str, vinca_conf: dict[str, Any], platform: str
) -> list[str]:
    configured = vinca_conf.get("_patches", {}).get(package_name)
    if not configured:
        return []
    return [*configured.get("any", []), *configured.get(platform.split("-")[0], [])]


def generate_source(
    distro: Any, vinca_conf: dict[str, Any], platform: str
) -> dict[str, dict]:
    """Generate per-package release sources for a multi-recipe build."""
    sources: dict[str, dict] = {}
    for shortname in vinca_conf["_selected_pkgs"]:
        if not distro.check_package(shortname):
            print(f"Could not generate source for {shortname}")
            continue
        if is_dummy_metapackage(shortname, vinca_conf):
            continue

        package_names = resolve_pkgname(shortname, vinca_conf, distro)
        version = distro.get_version(shortname)
        print("Checking ", shortname, version)
        if not package_names or _is_built(package_names[0], version, vinca_conf):
            continue

        url, ref, ref_type = distro.get_released_repo(shortname)
        package_name = package_names[0]
        entry = source_reference(url=url, ref=ref, ref_type=ref_type)
        entry["target_directory"] = f"{package_name}/src/work"

        patches = _package_patches(package_name, vinca_conf, platform)
        if patches:
            print(patches)
            common_root = os.path.commonpath([os.getcwd(), *patches])
            entry["patches"] = [
                os.path.relpath(patch, common_root) for patch in patches
            ]
        sources[package_name] = entry

    mutex_recipe = generate_mutex_package_recipe(vinca_conf, distro)
    if mutex_recipe:
        mutex = mutex_recipe["package"]
        if not should_skip_mutex_package(vinca_conf, mutex["name"], mutex["version"]):
            sources[mutex["name"]] = {}
    return sources


def generate_source_version(
    distro: Any, vinca_conf: dict[str, Any], platform: str
) -> dict[str, dict]:
    """Generate release sources while retaining absolute patch paths."""
    sources: dict[str, dict] = {}
    for shortname in vinca_conf["_selected_pkgs"]:
        if not distro.check_package(shortname):
            print(f"Could not generate source for {shortname}")
            continue

        package_names = resolve_pkgname(shortname, vinca_conf, distro)
        version = distro.get_version(shortname)
        if not package_names or _is_built(package_names[0], version, vinca_conf):
            continue

        url, ref, ref_type = distro.get_released_repo(shortname)
        package_name = package_names[0]
        entry = source_reference(url=url, ref=ref, ref_type=ref_type)
        entry["target_directory"] = f"{package_name}/src/work"
        if patches := _package_patches(package_name, vinca_conf, platform):
            entry["patches"] = patches
        sources[package_name] = entry
    return sources


def generate_fat_source(distro: Any, vinca_conf: dict[str, Any]) -> list[dict]:
    """Generate sources for a single fat recipe containing all packages."""
    sources = []
    for shortname in vinca_conf["_selected_pkgs"]:
        if not distro.check_package(shortname):
            print(f"Could not generate source for {shortname}")
            continue

        package_names = resolve_pkgname(shortname, vinca_conf, distro)
        if not package_names:
            continue
        url, ref, ref_type = distro.get_released_repo(shortname)
        package_name = package_names[0]
        entry = source_reference(url=url, ref=ref, ref_type=ref_type)
        entry["target_directory"] = f"src/{package_name}"

        patch_path = Path(vinca_conf["_patch_dir"]) / f"{package_name}.patch"
        if patch_path.exists():
            entry["patches"] = [f"{vinca_conf['patch_dir']}/{package_name}.patch"]
        sources.append(entry)
    return sources
