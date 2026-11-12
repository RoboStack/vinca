"""Schemas of the files vinca reads.

Every input file is validated into one of these models where it is read, so the rest
of vinca works with typed attributes instead of untyped mappings. The models of
vinca's own files reject unknown keys: a misspelled setting is an error rather than
silently ignored. Formats owned by other tools (conda-forge pinning, rosdep-style
conda_index files) only declare the keys vinca reads.
"""

from __future__ import annotations

from datetime import datetime
from enum import Enum
from functools import lru_cache
from pathlib import Path
from typing import TYPE_CHECKING, Annotated, Any, Optional, TypeVar, Union, cast

from pydantic import (
    BaseModel,
    BeforeValidator,
    ConfigDict,
    Field,
    TypeAdapter,
    ValidationError,
    model_validator,
)

if TYPE_CHECKING:
    from pydantic_core import InitErrorDetails

_T = TypeVar("_T")

# A recipe requirement: a match spec, or a conditional ``{if: ..., then: [...]}``.
Requirement = Union[str, dict[str, Any]]


class PackageNameMode(str, Enum):
    """Supported ROS package naming transition modes."""

    LEGACY = "legacy"
    BOTH = "both"
    NEW = "new"


class VariantsMode(Enum):
    """How generated recipes consume the repository-wide pinning configuration."""

    GLOBAL = "global"
    LOCAL = "local"


def _none_as_empty_list(value: Any) -> Any:
    return [] if value is None else value


def _none_as_empty_dict(value: Any) -> Any:
    return {} if value is None else value


def _package_names(value: Any) -> list[str]:
    """Package names of a (selector-resolved) package list, as written."""
    return [str(item) for item in value or [] if item is not None]


StrList = Annotated[list[str], BeforeValidator(_none_as_empty_list)]
PackageList = Annotated[list[str], BeforeValidator(_package_names)]
RequirementList = Annotated[list[Requirement], BeforeValidator(_none_as_empty_list)]


class _Schema(BaseModel):
    model_config = ConfigDict(extra="forbid", coerce_numbers_to_str=True)


class MutexPackage(_Schema):
    """``mutex_package`` as a mapping: vinca generates the distribution mutex."""

    name: str
    version: str
    upper_bound: str
    build_number: Optional[int] = None
    run_constraints: StrList = []


class DepMods(_Schema):
    """A package's entry in ``patch/dependencies.yaml``."""

    add_build: RequirementList = []
    add_host: RequirementList = []
    add_run: RequirementList = []
    remove_build: RequirementList = []
    remove_host: RequirementList = []
    remove_run: RequirementList = []


class DummyPackage(_Schema):
    """``generate_dummy_package_with_run_deps``: a recipe that only depends on the
    conda-forge package providing the ROS package."""

    dep_name: str
    upper_bound: Optional[str] = None
    max_pin: Optional[str] = None
    override_version: Optional[str] = None

    @model_validator(mode="after")
    def _has_upper_bound(self) -> DummyPackage:
        if not (self.upper_bound or self.max_pin):
            raise ValueError("Missing 'upper_bound' or 'max_pin'")
        return self

    @property
    def pin(self) -> str:
        return self.upper_bound or self.max_pin or ""


class PkgAdditionalInfo(_Schema):
    """A package's entry in ``pkg_additional_info.yaml``."""

    additional_cmake_args: Annotated[
        str, BeforeValidator(lambda value: "" if value is None else value)
    ] = ""
    build_number: Optional[int] = None
    generate_dummy_package_with_run_deps: Optional[DummyPackage] = None
    variant_overrides: Annotated[
        dict[str, Any], BeforeValidator(_none_as_empty_dict)
    ] = {}


class SnapshotEntry(_Schema):
    """A package of ``rosdistro_snapshot.yaml`` or ``rosdistro_additional_recipes.yaml``.

    ``dependencies`` is ``None`` for snapshots written before vinca-snapshot recorded
    them; the dependencies are then resolved from the rosdistro cache.
    """

    url: str
    version: str
    tag: Optional[str] = None
    rev: Optional[str] = None
    sha256: Optional[str] = None
    dependencies: Annotated[
        Optional[list[str]], BeforeValidator(_none_as_empty_list)
    ] = None
    repository: Optional[str] = None
    additional_folder: str = ""
    package_xml_name: str = "package.xml"


Snapshot = dict[str, SnapshotEntry]


class CondaIndexEntry(BaseModel):
    """A key of a conda_index file: the conda packages it maps to, for every platform
    or per platform (``linux``, ``osx``, ``win64``, ``emscripten`` or a conda subdir).

    conda_index files can be rosdep files with entries for other systems, so other
    keys are ignored.
    """

    model_config = ConfigDict(extra="ignore")

    robostack: Union[
        Annotated[list[str], BeforeValidator(_none_as_empty_list)],
        dict[str, Annotated[list[str], BeforeValidator(_none_as_empty_list)]],
        None,
    ] = None


CondaIndex = dict[str, CondaIndexEntry]


class RecipeTests(_Schema):
    """A package's ``tests/<package>.yaml``: tests added to its recipe."""

    tests: list[dict[str, Any]] = []


class VincaYaml(_Schema):
    """``vinca.yaml``, combined with the configurations it extends and with its
    selectors resolved for the target platform."""

    ros_distro: str
    python_version: Optional[str] = None
    build_number: Optional[int] = None
    package_name_mode: PackageNameMode = PackageNameMode.LEGACY
    variants_mode: VariantsMode = VariantsMode.GLOBAL
    conda_index: StrList = []
    patch_dir: Optional[str] = None
    rosdistro_snapshot: Optional[str] = None
    rosdistro_additional_recipes: Optional[str] = None
    mutex_package: Union[str, MutexPackage, None] = None
    build_all: bool = False
    packages_select_by_deps: PackageList = []
    packages_exclude: PackageList = []
    packages_skip: PackageList = []
    skip_existing: StrList = []
    trigger_new_versions: bool = False
    skip_testing: bool = True
    conditional_python_dependency: bool = False
    setup_pixi_version: Optional[str] = None
    pixi_version: Optional[str] = None
    build_in_own_stage: StrList = []
    build_in_own_azure_stage: StrList = []
    is_migration: bool = False
    # dependency fixes inline, for a configuration without patch/dependencies.yaml
    depmods: Annotated[dict[str, DepMods], BeforeValidator(_none_as_empty_dict)] = {}


class VincaYamlLayer(BaseModel):
    """The keys of one ``vinca.yaml``, before it is combined with the configurations
    it extends, that name files relative to it."""

    model_config = ConfigDict(extra="ignore")

    patch_dir: Optional[str] = None
    conda_index: StrList = []


class Provenance(BaseModel):
    """Why each package was selected: requested by the configuration, or required by
    the requested packages that (transitively) depend on it."""

    requested_by_config: set[str] = set()
    required_by: dict[str, set[str]] = {}


class VincaConfig(VincaYaml):
    """The configuration vinca works with: ``vinca.yaml`` and what
    :func:`vinca.configuration.read_vinca_yaml` derives from the files next to it.

    The fields below are never read from the user's file. The last group is filled in
    while recipes are generated.
    """

    skip_by_deps: list[str] = []
    remove_from_deps: list[str] = []
    patch_dir_path: Optional[Path] = None
    patches: dict[str, dict[str, list[str]]] = {}
    tests: dict[str, Path] = {}
    test_folders: dict[str, Path] = {}
    conda_indexes: list[CondaIndex] = []
    pkg_additional_info: dict[str, PkgAdditionalInfo] = {}
    variant_config: dict[str, Any] = {}
    snapshot: Snapshot = {}
    additional_packages_snapshot: Snapshot = {}

    selected_pkgs: list[str] = []
    skip_built_packages: set[Union[str, tuple[str, str]]] = set()
    pkg_provenance: Provenance = Provenance()

    @model_validator(mode="after")
    def _derive_exclusions(self) -> VincaConfig:
        """Without read_vinca_yaml's derived lists, derive them from the package
        lists (read_vinca_yaml also adds the deprecated keys' packages)."""
        if "skip_by_deps" not in self.model_fields_set:
            self.skip_by_deps = list(
                dict.fromkeys(self.packages_exclude + self.packages_skip)
            )
        if "remove_from_deps" not in self.model_fields_set:
            self.remove_from_deps = list(self.packages_exclude)
        return self


class PinningSpec(_Schema):
    """``vinca_pinning.yaml``: a conda-forge-pinning version, the migrations applied
    on top of it, and local overrides (``overrides`` is the old name)."""

    conda_forge_pinning_version: str
    migrations: list[str] = []
    pinning_overrides: Optional[dict[str, Any]] = None
    overrides: Optional[dict[str, Any]] = None


class BuildTimingsFile(_Schema):
    """Measured build times: ``{"recipes": {name: seconds}, "default": seconds,
    "job_overhead": seconds, "runners": jobs}``."""

    recipes: Annotated[dict[str, float], BeforeValidator(_none_as_empty_dict)] = {}
    default: Optional[float] = None
    job_overhead: Optional[float] = None
    runners: Optional[int] = None


class StageSettings(BaseModel):
    """The settings of ``vinca.yaml`` that CI pipeline batching reads."""

    model_config = ConfigDict(extra="ignore")

    build_in_own_stage: StrList = []
    build_in_own_azure_stage: StrList = []


class _RecipePackage(BaseModel):
    model_config = ConfigDict(coerce_numbers_to_str=True)

    name: str
    version: str


class _RecipeBuild(BaseModel):
    number: int = 0


class AdditionalRecipe(BaseModel):
    """A recipe of ``additional_recipes/``: the package it builds. The rest of the
    recipe is rattler-build's."""

    model_config = ConfigDict(extra="ignore")

    package: _RecipePackage
    build: _RecipeBuild = _RecipeBuild()


class Migration(BaseModel):
    """``migration.yaml`` of vinca-migrate: the packages being migrated."""

    model_config = ConfigDict(extra="ignore")

    packages: StrList = []


def _as_posix_timestamp(value: Any) -> Any:
    return value.timestamp() if isinstance(value, datetime) else value


class Migrator(BaseModel):
    """The ``__migrator`` block of a conda-forge migration (CFEP-9)."""

    model_config = ConfigDict(extra="ignore")

    operation: Optional[str] = None
    primary_key: Union[str, list[str], None] = None
    additional_zip_keys: StrList = []
    ordering: Annotated[dict[str, list[Any]], BeforeValidator(_none_as_empty_dict)] = {}
    paused: bool = False
    platform_allowlist: StrList = []


class MigrationFile(BaseModel):
    """A conda-forge migration file: ``__migrator``, ``migrator_ts``, and the
    variant keys it changes (read with ruamel, to keep their selector comments)."""

    model_config = ConfigDict(extra="ignore")

    migrator: Annotated[Migrator, BeforeValidator(_none_as_empty_dict)] = Field(
        Migrator(), validation_alias="__migrator"
    )
    migrator_ts: Annotated[float, BeforeValidator(_as_posix_timestamp)] = -1.0


class FeedstockMigrationStatus(BaseModel):
    """The migration PR of one feedstock."""

    model_config = ConfigDict(extra="ignore")

    pr_status: str = ""
    pr_url: str = ""


class MigrationStatus(BaseModel):
    """conda-forge-bot-data's ``status/migration_json/<migration>.json``: the
    feedstocks of a migration by state."""

    model_config = ConfigDict(extra="ignore")

    feedstock_status: dict[str, FeedstockMigrationStatus] = Field(
        {}, validation_alias="_feedstock_status"
    )
    done: StrList = []
    in_pr: StrList = Field([], validation_alias="in-pr")
    awaiting_pr: StrList = Field([], validation_alias="awaiting-pr")
    awaiting_parents: StrList = Field([], validation_alias="awaiting-parents")
    not_solvable: StrList = Field([], validation_alias="not-solvable")
    bot_error: StrList = Field([], validation_alias="bot-error")

    @property
    def states(self) -> dict[str, list[str]]:
        """The feedstocks by state, keyed as in the status file."""
        return {
            "done": self.done,
            "in-pr": self.in_pr,
            "awaiting-pr": self.awaiting_pr,
            "awaiting-parents": self.awaiting_parents,
            "not-solvable": self.not_solvable,
            "bot-error": self.bot_error,
        }

    @property
    def participants(self) -> list[str]:
        """Every feedstock the migration covers, whatever its state."""
        return [name for names in self.states.values() for name in names]


@lru_cache(maxsize=None)
def _adapter(schema: Any) -> TypeAdapter[Any]:
    return TypeAdapter(schema)


def validate(schema: type[_T], data: Any, source: Union[str, Path]) -> _T:
    """Validate parsed *data* of the file *source* (named in validation errors)."""
    try:
        return _adapter(schema).validate_python(data)
    except ValidationError as error:
        raise ValidationError.from_exception_data(
            str(source), cast("list[InitErrorDetails]", error.errors())
        ) from None
