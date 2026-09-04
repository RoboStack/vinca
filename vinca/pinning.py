"""Render and update the RoboStack pinning configuration.

``vinca_pinning.yaml`` deliberately contains only the provenance of the global
pinning file: an exact conda-forge-pinning package version, migrations applied
on top of it, and local overrides.  This module turns that small file into the
``conda_build_config.yaml`` consumed by rattler-build.
"""

from __future__ import annotations

import argparse
import io
import json
import os
import re
import sys
import tarfile
import zipfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator, Mapping, Optional, Sequence, Union
from urllib.parse import quote

import requests
import ruamel.yaml
import zstandard

from vinca.v1_selectors import _eval_condition, _platform_flags
from vinca.variant_algebra import variant_add

PINNING_PACKAGE = "conda-forge-pinning"
ANACONDA_RELEASE_URL = (
    "https://api.anaconda.org/release/conda-forge/conda-forge-pinning/{version}"
)
CURRENT_REPODATA_URL = (
    "https://conda.anaconda.org/conda-forge/noarch/current_repodata.json.zst"
)
MIGRATION_STATUS_URL = (
    "https://raw.githubusercontent.com/regro/cf-graph-countyfair/master/"
    "status/migration_json/{migration}.json"
)
FEEDSTOCK_OUTPUT_URL = (
    "https://raw.githubusercontent.com/conda-forge/feedstock-outputs/main/"
    "outputs/{shards}/{package}.json"
)
DEFAULT_PLATFORMS = ("linux-64", "osx-64", "win-64")
STATUS_CATEGORIES = (
    "done",
    "in-pr",
    "awaiting-pr",
    "awaiting-parents",
    "not-solvable",
    "bot-error",
)


class PinningError(RuntimeError):
    """A pinning file could not be downloaded, interpreted, or updated."""


def _yaml() -> Any:
    yaml = ruamel.yaml.YAML()
    yaml.width = 4096
    yaml.indent(mapping=2, sequence=4, offset=2)
    return yaml


def _request(url: str, *, timeout: int = 120) -> Any:
    try:
        response = requests.get(url, timeout=timeout)
        response.raise_for_status()
    except requests.RequestException as exc:
        raise PinningError(f"Could not download {url}: {exc}") from exc
    return response


def _load_zstd_json(payload: bytes) -> Any:
    try:
        decompressed = zstandard.ZstdDecompressor().decompress(payload)
    except zstandard.ZstdError as exc:
        raise PinningError("Could not decompress conda-forge repodata") from exc
    return json.loads(decompressed)


def get_latest_pinning_distribution() -> tuple[str, str]:
    """Return the newest conda-forge-pinning version and artifact URL."""
    repodata = _load_zstd_json(_request(CURRENT_REPODATA_URL).content)
    records = []
    for section in ("packages", "packages.conda"):
        for filename, record in repodata.get(section, {}).items():
            if record.get("name") == PINNING_PACKAGE:
                records.append((record.get("timestamp", 0), filename, record))
    if not records:
        raise PinningError(
            f"No {PINNING_PACKAGE} package found in conda-forge current repodata"
        )
    _, filename, record = max(records)
    url = f"https://conda.anaconda.org/conda-forge/noarch/{quote(filename)}"
    return str(record["version"]), url


def get_pinning_distribution(version: str) -> str:
    """Return the artifact URL for an exact conda-forge-pinning version."""
    release = _request(ANACONDA_RELEASE_URL.format(version=quote(str(version)))).json()
    distributions = [
        item
        for item in release.get("distributions", [])
        if item.get("basename", "").startswith("noarch/")
        and item.get("basename", "").endswith((".conda", ".tar.bz2"))
    ]
    if not distributions:
        raise PinningError(f"No noarch artifact found for {PINNING_PACKAGE} {version}")
    distribution = max(
        distributions,
        key=lambda item: (
            item.get("attrs", {}).get("build_number", 0),
            item.get("attrs", {}).get("timestamp", 0),
        ),
    )
    url = distribution.get("download_url")
    if not url:
        basename = distribution["basename"].split("/", 1)[1]
        url = "https://conda.anaconda.org/conda-forge/noarch/" + quote(basename)
    elif url.startswith("//"):
        url = "https:" + url
    return url


def _read_tar_members(fileobj: Any, *, mode: str) -> dict[str, bytes]:
    """Extract only the base config and migration YAML files from a package tarball."""
    files = {}
    with tarfile.open(fileobj=fileobj, mode=mode) as archive:
        for member in archive:
            if not member.isfile():
                continue
            name = member.name.lstrip("./")
            if name == "conda_build_config.yaml" or (
                name.startswith("share/conda-forge/migrations/")
                and name.endswith(".yaml")
            ):
                extracted = archive.extractfile(member)
                if extracted is not None:
                    files[name] = extracted.read()
    return files


def _read_pinning_package(payload: bytes) -> tuple[bytes, dict[str, bytes]]:
    """Parse supported pinning artifacts into base-config bytes and named migrations.

    Raises ``PinningError`` when the artifact lacks the required package members.
    """
    stream = io.BytesIO(payload)
    if zipfile.is_zipfile(stream):
        with zipfile.ZipFile(stream) as archive:
            package_members = [
                name
                for name in archive.namelist()
                if name.startswith("pkg-") and name.endswith(".tar.zst")
            ]
            if len(package_members) != 1:
                raise PinningError("Invalid .conda artifact: package archive not found")
            compressed = archive.read(package_members[0])
        reader = zstandard.ZstdDecompressor().stream_reader(io.BytesIO(compressed))
        with reader:
            files = _read_tar_members(reader, mode="r|")
    else:
        try:
            files = _read_tar_members(io.BytesIO(payload), mode="r:bz2")
        except tarfile.TarError as exc:
            raise PinningError("Unsupported conda-forge-pinning artifact") from exc

    base = files.pop("conda_build_config.yaml", None)
    if base is None:
        raise PinningError("conda_build_config.yaml is missing from pinning artifact")
    migrations = {
        Path(name).stem: content
        for name, content in files.items()
        if Path(name).suffix == ".yaml"
    }
    return base, migrations


def download_pinning_package(
    version: str, *, artifact_url: Optional[str] = None
) -> tuple[bytes, dict[str, bytes]]:
    """Download and parse a versioned pinning artifact, or the supplied artifact URL."""
    url = artifact_url or get_pinning_distribution(version)
    return _read_pinning_package(_request(url).content)


def _pinning_spec_fields(
    data: Mapping[str, Any],
) -> tuple[str, list[str], Mapping[str, Any]]:
    """Validate and normalize version, migration, and override fields from a pinning spec."""
    version = data.get("conda_forge_pinning_version")
    migrations = data.get("migrations", [])
    overrides = data.get("pinning_overrides")
    if overrides is None:
        overrides = data.get("overrides", {})
    if version is None:
        raise PinningError("Missing 'conda_forge_pinning_version'")
    if not isinstance(migrations, list) or not all(
        isinstance(item, str) for item in migrations
    ):
        raise PinningError("'migrations' must be a list of migration names")
    if not isinstance(overrides, dict):
        raise PinningError("'pinning_overrides' must be a mapping")
    return str(version), migrations, overrides


def _migration_name(name: str) -> str:
    """Return a safe migration stem, rejecting path-like or malformed names."""
    name = name.removesuffix(".yaml")
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", name):
        raise PinningError(f"Invalid migration name: {name!r}")
    return name


def _overlay(target: Any, source: Any) -> None:
    for key, value in source.items():
        if key == "migrator_ts" or str(key).startswith("__"):
            continue
        target[key] = value


def _migration_timestamp(payload: bytes) -> float:
    data = _yaml().load(payload.decode("utf-8")) or {}
    return float(data.get("migrator_ts", -1.0))


def _validate_zipped_overrides(rendered: Any, overrides: Any) -> None:
    """Reject partial overrides of zip-key groups that would break value alignment."""
    override_keys = set(overrides)
    for group in rendered.get("zip_keys", []):
        group = set(group)
        touched = group & override_keys
        missing = group - override_keys
        if touched and missing:
            raise PinningError(
                "Pinning overrides must update an entire zip_keys group; "
                f"{', '.join(sorted(touched))} also requires "
                f"{', '.join(sorted(missing))}"
            )


def _format_rendered_yaml(payload: str) -> str:
    """Normalize emitted YAML while preserving selectors on empty list items."""

    lines = payload.splitlines()
    formatted = []
    index = 0
    while index < len(lines):
        line = lines[index].rstrip(" \t")
        if re.fullmatch(r"\s*-", line) and index + 1 < len(lines):
            following = lines[index + 1].rstrip(" \t")
            comment = re.fullmatch(r"(\s+)(#.*)", following)
            if comment and len(comment.group(1)) > len(line):
                formatted.append(
                    line + " " * (len(comment.group(1)) - len(line)) + comment.group(2)
                )
                index += 2
                continue
        formatted.append(line)
        index += 1
    return "\n".join(formatted) + "\n"


def render_pinning(
    config_path: Union[str, Path],
    output_path: Union[str, Path],
    *,
    package: Optional[tuple[bytes, Mapping[str, bytes]]] = None,
) -> Any:
    """Render a pinning spec by applying ordered migrations and local overrides.

    Raises ``PinningError`` for invalid inputs, unavailable selected migrations, or failed merges.
    """
    yaml = _yaml()
    config_path = Path(config_path)
    output_path = Path(output_path)
    try:
        with config_path.open(encoding="utf-8") as stream:
            spec = yaml.load(stream) or {}
    except OSError as exc:
        raise PinningError(f"Could not read {config_path}: {exc}") from exc
    version, selected_migrations, overrides = _pinning_spec_fields(spec)
    if package is None:
        base_payload, migration_payloads = download_pinning_package(version)
    else:
        base_payload, migration_payloads = package

    rendered = yaml.load(base_payload.decode("utf-8")) or {}
    migration_names = {_migration_name(name): name for name in selected_migrations}
    missing = sorted(set(migration_names) - set(migration_payloads))
    if missing:
        raise PinningError(
            "Migrations not present in conda-forge-pinning "
            f"{version}: {', '.join(missing)}"
        )
    ordered_migrations = sorted(
        migration_names,
        key=lambda name: (_migration_timestamp(migration_payloads[name]), name),
    )
    for migration in ordered_migrations:
        migration_config = yaml.load(migration_payloads[migration].decode("utf-8"))
        try:
            rendered = variant_add(rendered, migration_config or {})
        except (KeyError, RuntimeError, TypeError, ValueError) as exc:
            raise PinningError(f"Could not apply migration {migration}: {exc}") from exc
    _validate_zipped_overrides(rendered, overrides)
    _overlay(rendered, overrides)
    rendered.yaml_set_start_comment(
        f"Generated by vinca-pinning-render from {config_path.name}.\n"
        "Do not edit this file directly."
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    stream = io.StringIO()
    yaml.dump(rendered, stream)
    output_path.write_text(_format_rendered_yaml(stream.getvalue()), encoding="utf-8")
    return rendered


def _dependency_name(requirement: Any) -> Optional[str]:
    if not isinstance(requirement, str) or requirement.startswith("${{"):
        return None
    name = requirement.split()[0]
    if name.startswith(("ros-", "ros2-")):
        return None
    return name


def _walk_requirements(value: Any) -> Iterator[str]:
    """Yield requirement strings recursively, following both conditional branches."""
    if isinstance(value, str):
        yield value
    elif isinstance(value, list):
        for item in value:
            yield from _walk_requirements(item)
    elif isinstance(value, dict):
        if "if" in value and "then" in value:
            branches = (value.get("then"), value.get("else"))
        else:
            branches = value.values()
        for item in branches:
            yield from _walk_requirements(item)


def dependencies_from_recipes(recipe_dir: Union[str, Path]) -> set[str]:
    """Collect non-ROS conda dependency names from every generated recipe."""
    yaml = _yaml()
    dependencies = set()
    for recipe_path in Path(recipe_dir).glob("**/recipe.yaml"):
        with recipe_path.open(encoding="utf-8") as stream:
            recipe = yaml.load(stream) or {}
        for requirement in _walk_requirements(recipe.get("requirements", {})):
            if name := _dependency_name(requirement):
                dependencies.add(name)
    return dependencies


@contextmanager
def _working_directory(path: Any) -> Iterator[None]:
    previous = Path.cwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(previous)


def dependencies_from_vinca(
    base_dir: Union[str, Path], platforms: Sequence[str] = DEFAULT_PLATFORMS
) -> set[str]:
    """Generate non-ROS dependencies for each platform while preserving global config state.

    Raises ``PinningError`` if platform selection changes the configured ROS snapshots.
    """
    from vinca import config
    from vinca.distro import Distro
    from vinca.main import (
        generate_dependency_requirements,
        get_group_dependency_packages,
        get_selected_packages,
        read_vinca_yaml,
    )

    base_dir = Path(base_dir).resolve()
    dependencies = set()
    distro = None
    group_packages = None
    previous_platform = config.selected_platform
    previous_args = config.parsed_args
    try:
        with _working_directory(base_dir):
            for platform in platforms:
                config.selected_platform = platform
                config.parsed_args = argparse.Namespace(platform=platform)
                vinca_config = read_vinca_yaml(base_dir / "vinca.yaml")
                vinca_config["skip_built_packages"] = []
                if distro is None:
                    distro = Distro(
                        vinca_config["ros_distro"],
                        vinca_config.get("python_version"),
                        vinca_config["_snapshot"],
                        vinca_config["_additional_packages_snapshot"],
                    )
                    distro.prefetch_additional_package_xml()
                    group_packages = get_group_dependency_packages(distro)
                elif (
                    distro.name != vinca_config["ros_distro"]
                    or distro.snapshot != vinca_config["_snapshot"]
                    or distro.additional_packages_snapshot
                    != vinca_config["_additional_packages_snapshot"]
                ):
                    raise PinningError(
                        "Platform selectors must not change the ROS distro snapshots"
                    )
                vinca_config["_selected_pkgs"] = get_selected_packages(
                    distro, vinca_config
                )
                for requirement_group in generate_dependency_requirements(
                    distro, vinca_config, group_packages
                ):
                    for requirement in _walk_requirements(requirement_group):
                        if name := _dependency_name(requirement):
                            dependencies.add(name)
    finally:
        config.selected_platform = previous_platform
        config.parsed_args = previous_args
    return dependencies


def _normalized(name: str) -> str:
    return name.lower().replace("_", "-")


def _feedstock_output_url(package: str) -> str:
    prefix = (package.lower() + "zzz")[:3]
    shards = "/".join(prefix)
    return FEEDSTOCK_OUTPUT_URL.format(
        shards=shards, package=quote(package.lower(), safe="")
    )


def package_feedstocks(package: str) -> set[str]:
    """Look up package-producing feedstocks, falling back to the package name on lookup failure."""
    url = _feedstock_output_url(package)
    try:
        response = requests.get(url, timeout=30)
        if response.status_code == 404:
            return {package}
        response.raise_for_status()
        feedstocks = response.json().get("feedstocks", [])
    except (requests.RequestException, ValueError):
        return {package}
    return set(feedstocks) or {package}


def get_migration_status(migration: str) -> Optional[dict[str, Any]]:
    """Fetch migration status, returning ``None`` only when no status record exists."""
    url = MIGRATION_STATUS_URL.format(migration=quote(migration.lower(), safe=""))
    try:
        response = requests.get(url, timeout=60)
        if response.status_code == 404:
            return None
        response.raise_for_status()
        return response.json()
    except (requests.RequestException, ValueError) as exc:
        raise PinningError(
            f"Could not read conda-forge status for migration {migration}: {exc}"
        ) from exc


def _migration_pin_keys(payload: bytes) -> set[str]:
    data = _yaml().load(payload.decode("utf-8")) or {}
    return {
        _normalized(str(key))
        for key in data
        if key != "migrator_ts" and not str(key).startswith("__")
    }


def _is_paused_migration(payload: bytes) -> bool:
    data = _yaml().load(payload.decode("utf-8")) or {}
    return bool((data.get("__migrator") or {}).get("paused", False))


def _comment_selector(comment: Any) -> Optional[str]:
    if comment is None:
        return None
    match = re.search(r"#\s*\[(.+)]", comment.value)
    return match.group(1) if match else None


def _migration_selectors(data: Any) -> Iterator[Optional[str]]:
    """Yield selector expressions for each top-level variant value."""
    for key, value in data.items():
        if key == "migrator_ts" or str(key).startswith("__"):
            continue
        key_comment = data.ca.items.get(key, [None, None, None])[2]
        key_selector = _comment_selector(key_comment)
        if isinstance(value, list):
            for index in range(len(value)):
                item_comment = value.ca.items.get(index, [None])[0]
                item_selector = _comment_selector(item_comment)
                if key_selector and item_selector:
                    yield f"({key_selector}) and ({item_selector})"
                else:
                    yield key_selector or item_selector
        else:
            yield key_selector


def _migration_applies_to_platforms(payload: bytes, platforms: Sequence[str]) -> bool:
    """Return whether migration allowlists or selectors affect any requested platform."""
    data = _yaml().load(payload.decode("utf-8")) or {}
    migrator = data.get("__migrator") or {}
    allowlist = set(migrator.get("platform_allowlist", []))
    if allowlist and not allowlist.intersection(platforms):
        return False
    selectors = list(_migration_selectors(data))
    if not selectors or any(selector is None for selector in selectors):
        return True
    return any(
        _eval_condition(selector, _platform_flags(platform))
        for selector in selectors
        for platform in platforms
    )


def _status_sets(status: Any) -> tuple[set[str], set[str]]:
    values = {
        category: {_normalized(name) for name in status.get(category, [])}
        for category in STATUS_CATEGORIES
    }
    participants = set().union(*values.values())
    return values["done"], participants


def select_completed_migrations(
    migration_payloads: Mapping[str, bytes],
    dependencies: set[str],
    existing_migrations: Sequence[str] = (),
    platforms: Optional[Sequence[str]] = None,
) -> tuple[list[str], list[tuple[str, str]]]:
    """Select applicable active migrations complete for relevant dependency feedstocks.

    Already-applied migrations remain selected, including paused or platform-inapplicable ones.
    """
    existing = {_migration_name(name) for name in existing_migrations}
    candidates = {
        name: payload
        for name, payload in migration_payloads.items()
        if not _is_paused_migration(payload) or name in existing
        if platforms is None
        or name in existing
        or _migration_applies_to_platforms(payload, platforms)
    }

    statuses = {}
    with ThreadPoolExecutor(max_workers=12) as executor:
        futures = {
            executor.submit(get_migration_status, name): name for name in candidates
        }
        for future in as_completed(futures):
            statuses[futures[future]] = future.result()

    all_participants = set()
    for status in statuses.values():
        if status is not None:
            _, participants = _status_sets(status)
            all_participants.update(participants)

    direct_feedstocks = {}
    unmatched = [
        dependency
        for dependency in dependencies
        if _normalized(dependency) not in all_participants
    ]
    with ThreadPoolExecutor(max_workers=20) as executor:
        futures = {
            executor.submit(package_feedstocks, dependency): dependency
            for dependency in unmatched
        }
        for future in as_completed(futures):
            direct_feedstocks[futures[future]] = future.result()

    dependency_feedstocks = {_normalized(name) for name in dependencies}
    for feedstocks in direct_feedstocks.values():
        dependency_feedstocks.update(_normalized(name) for name in feedstocks)

    selected = []
    reports = []
    normalized_dependencies = {_normalized(name) for name in dependencies}
    for name in sorted(candidates):
        status = statuses.get(name)
        if name in existing:
            selected.append(name)
            reports.append((name, "kept (already applied)"))
            continue
        if status is None:
            reports.append((name, "skipped (status unavailable)"))
            continue
        done, participants = _status_sets(status)
        relevant = dependency_feedstocks & participants
        directly_pinned = normalized_dependencies & _migration_pin_keys(
            candidates[name]
        )
        pending = relevant - done
        if not pending and (relevant or directly_pinned):
            selected.append(name)
            reports.append((name, f"selected ({len(relevant)} dependencies done)"))
        elif pending:
            sample = ", ".join(sorted(pending)[:5])
            reports.append((name, f"waiting for {sample}"))
        else:
            reports.append((name, "not relevant to generated recipes"))
    return selected, reports


def update_pinning(
    config_path: Union[str, Path],
    *,
    base_dir: Optional[Union[str, Path]] = None,
    platforms: Sequence[str] = DEFAULT_PLATFORMS,
    dependencies: Optional[set[str]] = None,
    latest_distribution: Optional[tuple[str, str]] = None,
) -> tuple[str, list[str], list[tuple[str, str]]]:
    """Update a pinning spec to the latest base package and selected migrations.

    Dependency discovery uses existing recipes when available, otherwise generates them from Vinca.
    """
    config_path = Path(config_path)
    base_dir = Path(base_dir or config_path.parent)
    yaml = _yaml()
    if config_path.exists():
        with config_path.open(encoding="utf-8") as stream:
            spec = yaml.load(stream) or {}
        _, existing_migrations, _ = _pinning_spec_fields(spec)
    else:
        spec = ruamel.yaml.comments.CommentedMap()
        spec["conda_forge_pinning_version"] = ""
        spec["migrations"] = ruamel.yaml.comments.CommentedSeq()
        spec["pinning_overrides"] = ruamel.yaml.comments.CommentedMap()
        existing_migrations = []

    version, artifact_url = latest_distribution or get_latest_pinning_distribution()
    _, migration_payloads = download_pinning_package(version, artifact_url=artifact_url)
    if dependencies is None:
        recipe_dir = base_dir / "recipes"
        if recipe_dir.is_dir() and any(recipe_dir.glob("**/recipe.yaml")):
            dependencies = dependencies_from_recipes(recipe_dir)
        else:
            dependencies = dependencies_from_vinca(base_dir, platforms)
    migrations, reports = select_completed_migrations(
        migration_payloads,
        set(dependencies),
        existing_migrations,
        platforms=platforms,
    )

    # Canonicalize legacy field names while leaving the override mapping untouched.
    spec["conda_forge_pinning_version"] = version
    spec["migrations"] = migrations
    if "overrides" in spec and "pinning_overrides" not in spec:
        spec["pinning_overrides"] = spec.pop("overrides")
    spec.setdefault("pinning_overrides", ruamel.yaml.comments.CommentedMap())
    with config_path.open("w", encoding="utf-8") as stream:
        yaml.dump(spec, stream)
    return version, migrations, reports


def _common_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        "-d", "--dir", default=".", help="RoboStack repository directory"
    )
    parser.add_argument(
        "--config",
        default="vinca_pinning.yaml",
        help="pinning specification path, relative to --dir",
    )
    return parser


def render_main(argv: Optional[Sequence[str]] = None) -> None:
    parser = _common_parser("Render conda_build_config.yaml from vinca pinning")
    parser.add_argument(
        "-o",
        "--output",
        default="conda_build_config.yaml",
        help="output path, relative to --dir",
    )
    args = parser.parse_args(argv)
    base_dir = Path(args.dir)
    try:
        render_pinning(base_dir / args.config, base_dir / args.output)
    except PinningError as exc:
        parser.error(str(exc))
    print(f"Rendered {base_dir / args.output}")


def update_main(argv: Optional[Sequence[str]] = None) -> None:
    parser = _common_parser("Update vinca pinning from conda-forge migration status")
    parser.add_argument(
        "--platform",
        action="append",
        choices=("linux-64", "linux-aarch64", "osx-64", "osx-arm64", "win-64"),
        help="platform to inspect (repeatable; defaults to Linux, macOS, and Windows)",
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="also regenerate conda_build_config.yaml after updating",
    )
    args = parser.parse_args(argv)
    base_dir = Path(args.dir)
    config_path = base_dir / args.config
    try:
        version, migrations, reports = update_pinning(
            config_path,
            base_dir=base_dir,
            platforms=tuple(args.platform or DEFAULT_PLATFORMS),
        )
        if args.render:
            render_pinning(config_path, base_dir / "conda_build_config.yaml")
    except PinningError as exc:
        parser.error(str(exc))
    print(f"Updated {config_path} to conda-forge-pinning {version}")
    print("Applied migrations: " + (", ".join(migrations) or "none"))
    for migration, report in reports:
        print(f"  {migration}: {report}")


if __name__ == "__main__":
    sys.exit(render_main())
