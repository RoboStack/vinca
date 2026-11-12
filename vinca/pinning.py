"""Render and update the RoboStack pinning configuration.

``vinca_pinning.yaml`` deliberately contains only the provenance of the global
pinning file: an exact conda-forge-pinning package version, migrations applied
on top of it, and local overrides.  This module turns that small file into the
``conda_build_config.yaml`` consumed by rattler-build.
"""

from __future__ import annotations

import argparse
import asyncio
import io
import os
import re
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import contextmanager
from pathlib import Path
from typing import (
    TYPE_CHECKING,
    Any,
    Iterator,
    Mapping,
    Optional,
    Sequence,
    Union,
    cast,
)
from urllib.parse import quote

import rattler
import requests
import ruamel.yaml
from conda_forge_metadata.feedstock_outputs import package_to_feedstock
from pydantic import ValidationError
from rattler.exceptions import ExtractError, GatewayError
from rattler.package_streaming import download_and_extract

from vinca.schemas import (
    MigrationFile,
    MigrationStatus,
    PinningSpec,
    VincaConfig,
    validate,
)
from vinca.v1_selectors import _eval_condition, _platform_flags
from vinca.variant_algebra import variant_add

if TYPE_CHECKING:
    from vinca.distro import Distro

PINNING_PACKAGE = "conda-forge-pinning"
MIGRATION_STATUS_URL = (
    "https://raw.githubusercontent.com/conda-forge/conda-forge-bot-data/main/"
    "status/migration_json/{migration}.json"
)
DEFAULT_PLATFORMS = ("linux-64", "osx-64", "win-64")


class PinningError(RuntimeError):
    """A pinning file could not be downloaded, interpreted, or updated."""


def _yaml() -> Any:
    yaml = ruamel.yaml.YAML()
    yaml.width = 4096
    yaml.indent(mapping=2, sequence=4, offset=2)
    return yaml


def _newest_pinning_record(spec: str) -> Any:
    """The newest noarch conda-forge record matching *spec*."""
    try:
        [records] = asyncio.run(
            rattler.Gateway().query(
                [rattler.Channel("conda-forge")], ["noarch"], [spec], recursive=False
            )
        )
    except GatewayError as exc:
        raise PinningError(f"Could not query conda-forge for {spec}: {exc}") from exc
    if not records:
        raise PinningError(f"No {spec} package found on conda-forge")
    return max(records, key=lambda record: (record.version, record.build_number))


def get_latest_pinning_distribution() -> tuple[str, str]:
    """Return the newest conda-forge-pinning version and artifact URL."""
    record = _newest_pinning_record(PINNING_PACKAGE)
    return str(record.version), record.url


def get_pinning_distribution(version: str) -> str:
    """Return the artifact URL for an exact conda-forge-pinning version."""
    return _newest_pinning_record(f"{PINNING_PACKAGE}=={version}").url


def download_pinning_package(
    version: str, *, artifact_url: Optional[str] = None
) -> tuple[bytes, dict[str, bytes]]:
    """Download a versioned pinning artifact, or the supplied artifact URL, into its
    base config and named migrations."""
    url = artifact_url or get_pinning_distribution(version)
    with tempfile.TemporaryDirectory() as tmp:
        dest = Path(tmp)
        try:
            asyncio.run(download_and_extract(rattler.Client(), url, dest))
        except (ExtractError, OSError) as exc:
            raise PinningError(f"Could not download {url}: {exc}") from exc
        base = dest / "conda_build_config.yaml"
        if not base.is_file():
            raise PinningError(
                "conda_build_config.yaml is missing from pinning artifact"
            )
        migrations = {
            path.stem: path.read_bytes()
            for path in (dest / "share/conda-forge/migrations").glob("*.yaml")
        }
        return base.read_bytes(), migrations


def _pinning_spec_fields(
    data: Mapping[str, Any], source: Union[str, Path]
) -> tuple[str, list[str], Mapping[str, Any]]:
    """Validate and normalize version, migration, and override fields from a pinning spec."""
    try:
        spec = validate(PinningSpec, data, source)
    except ValidationError as exc:
        raise PinningError(str(exc)) from exc
    overrides = spec.pinning_overrides
    if overrides is None:
        overrides = spec.overrides or {}
    return spec.conda_forge_pinning_version, spec.migrations, overrides


def _migration_name(name: str) -> str:
    """Return a safe migration stem, rejecting path-like or malformed names."""
    name = name.removesuffix(".yaml")
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", name):
        raise PinningError(f"Invalid migration name: {name!r}")
    return name


def _overlay(target: Any, source: Any) -> None:
    removed_keys = {key for key, value in source.items() if value is None}
    for key, value in source.items():
        if key == "migrator_ts" or str(key).startswith("__"):
            continue
        if value is None:
            target.pop(key, None)
        else:
            target[key] = value
    if removed_keys and "zip_keys" in target:
        groups = [
            group for group in target["zip_keys"] if not set(group) <= removed_keys
        ]
        if groups:
            target["zip_keys"] = groups
        else:
            target.pop("zip_keys")


def _migration(payload: bytes, name: str = "migration") -> tuple[Any, MigrationFile]:
    """The ruamel data of a migration file, and its validated ``__migrator`` fields."""
    data = _yaml().load(payload.decode("utf-8")) or {}
    try:
        return data, validate(MigrationFile, data, name)
    except ValidationError as exc:
        raise PinningError(str(exc)) from exc


def _migration_timestamp(payload: bytes) -> float:
    return _migration(payload)[1].migrator_ts


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
        removed = {key for key in group if key in overrides and overrides[key] is None}
        if removed and removed != group:
            raise PinningError(
                "Pinning overrides must remove every member of a zip_keys group; "
                f"{', '.join(sorted(removed))} also requires "
                f"{', '.join(sorted(group - removed))}: null"
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
    version, selected_migrations, overrides = _pinning_spec_fields(spec, config_path)
    if package is None:
        base_payload, migration_payloads = download_pinning_package(version)
    else:
        base_payload, migration_payloads = package

    rendered: Any = yaml.load(base_payload.decode("utf-8")) or {}
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
            rendered = cast(Any, variant_add(rendered, migration_config or {}))
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


@contextmanager
def _platform_configurations(
    base_dir: Union[str, Path], platforms: Sequence[str]
) -> Iterator[Iterator[tuple[str, Distro, VincaConfig, Any]]]:
    """Yield ``(platform, distro, vinca_config, group_packages)`` for each platform.

    The global vinca config state is restored afterwards. Raises ``PinningError`` if
    platform selection changes the configured ROS snapshots.
    """
    from vinca import config
    from vinca.distro import Distro
    from vinca.main import (
        get_group_dependency_packages,
        get_selected_packages,
        read_vinca_yaml,
    )

    base_dir = Path(base_dir).resolve()
    previous_platform = config.selected_platform
    previous_args = config.parsed_args

    def configurations() -> Iterator[tuple[str, Distro, VincaConfig, Any]]:
        distro = None
        group_packages = None
        for platform in platforms:
            config.selected_platform = platform
            config.parsed_args = argparse.Namespace(platform=platform)
            vinca_config = read_vinca_yaml(base_dir / "vinca.yaml")
            if distro is None:
                distro = Distro(
                    vinca_config.ros_distro,
                    vinca_config.python_version,
                    vinca_config.snapshot,
                    vinca_config.additional_packages_snapshot,
                )
                distro.prefetch_additional_package_xml()
                group_packages = get_group_dependency_packages(distro)
            elif (
                distro.name != vinca_config.ros_distro
                or distro.snapshot != vinca_config.snapshot
                or distro.additional_packages_snapshot
                != vinca_config.additional_packages_snapshot
            ):
                raise PinningError(
                    "Platform selectors must not change the ROS distro snapshots"
                )
            if group_packages is None:
                raise PinningError("No group packages were generated")
            vinca_config.selected_pkgs = get_selected_packages(distro, vinca_config)
            yield platform, distro, vinca_config, group_packages

    try:
        with _working_directory(base_dir):
            yield configurations()
    finally:
        config.selected_platform = previous_platform
        config.parsed_args = previous_args


def dependencies_from_vinca(
    base_dir: Union[str, Path], platforms: Sequence[str] = DEFAULT_PLATFORMS
) -> set[str]:
    """Generate non-ROS dependencies for each platform while preserving global config state.

    Raises ``PinningError`` if platform selection changes the configured ROS snapshots.
    """
    from vinca.main import generate_dependency_requirements

    dependencies = set()
    with _platform_configurations(base_dir, platforms) as configurations:
        for _, distro, vinca_config, group_packages in configurations:
            for requirement_group in generate_dependency_requirements(
                distro, vinca_config, group_packages
            ):
                for requirement in _walk_requirements(requirement_group):
                    if name := _dependency_name(requirement):
                        dependencies.add(name)
    return dependencies


def _normalized(name: str) -> str:
    return name.lower().replace("_", "-")


def package_feedstocks(package: str) -> set[str]:
    """Look up package-producing feedstocks, falling back to the package name on lookup failure."""
    try:
        feedstocks = package_to_feedstock(package, timeout=30)
    except (requests.RequestException, ValueError, KeyError):
        return {package}
    return set(feedstocks) or {package}


def get_migration_status(migration: str) -> Optional[MigrationStatus]:
    """Fetch migration status, returning ``None`` only when no status record exists."""
    url = MIGRATION_STATUS_URL.format(migration=quote(migration.lower(), safe=""))
    try:
        response = requests.get(url, timeout=60)
        if response.status_code == 404:
            return None
        response.raise_for_status()
        return validate(MigrationStatus, response.json(), url)
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
    return _migration(payload)[1].migrator.paused


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
            value_with_comments: Any = value
            for index in range(len(value)):
                item_comment = value_with_comments.ca.items.get(index, [None])[0]
                item_selector = _comment_selector(item_comment)
                if key_selector and item_selector:
                    yield f"({key_selector}) and ({item_selector})"
                else:
                    yield key_selector or item_selector
        else:
            yield key_selector


def _migration_applies_to_platforms(payload: bytes, platforms: Sequence[str]) -> bool:
    """Return whether migration allowlists or selectors affect any requested platform."""
    data, migration = _migration(payload)
    allowlist = set(migration.migrator.platform_allowlist)
    if allowlist and not allowlist.intersection(platforms):
        return False
    selectors = list(_migration_selectors(data))
    if not selectors or any(selector is None for selector in selectors):
        return True
    return any(
        _eval_condition(selector, _platform_flags(platform))
        for selector in selectors
        if selector is not None
        for platform in platforms
    )


def _status_sets(status: MigrationStatus) -> tuple[set[str], set[str]]:
    done = {_normalized(name) for name in status.done}
    participants = {_normalized(name) for name in status.participants}
    return done, participants


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
        _, existing_migrations, _ = _pinning_spec_fields(spec, config_path)
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
