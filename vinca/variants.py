"""Variant configuration helpers for generated rattler-build recipes."""

from __future__ import annotations

import copy
import re
from enum import Enum
from typing import Any, Mapping, Sequence


class VariantsMode(Enum):
    """How generated recipes consume the repository-wide pinning configuration."""

    GLOBAL = "global"
    LOCAL = "local"


def get_variants_mode(vinca_conf: Mapping[str, Any]) -> VariantsMode:
    """Return the configured variants mode, defaulting to the global mode."""
    value = vinca_conf.get("variants_mode", VariantsMode.GLOBAL.value)
    try:
        return VariantsMode(value)
    except ValueError as exc:
        choices = ", ".join(mode.value for mode in VariantsMode)
        raise ValueError(
            f"Invalid variants_mode {value!r}; expected one of: {choices}"
        ) from exc


_SELECTOR_RE = re.compile(r"#\s*\[(?P<condition>[^]]+)]")
_COMPILER_RE = re.compile(r"\bcompiler\(\s*['\"](?P<language>[^'\"]+)['\"]\s*\)")
_STDLIB_RE = re.compile(r"\bstdlib\(\s*['\"](?P<language>[^'\"]+)['\"]\s*\)")
_JINJA_NAME_RE = re.compile(r"(?<![\w-])(?P<name>[A-Za-z_][A-Za-z0-9_]*)(?![\w-])")
_SPECIAL_VARIANT_KEYS = {"zip_keys", "pin_run_as_build"}


def _selector_from_comment(comment: Any) -> str | None:
    """Extract a v0 selector from ruamel's comment metadata."""
    if comment is None:
        return None
    if isinstance(comment, (list, tuple)):
        for item in comment:
            if selector := _selector_from_comment(item):
                return selector
        return None
    match = _SELECTOR_RE.search(str(getattr(comment, "value", comment)))
    return match.group("condition").strip() if match else None


def _combine_selectors(parent: str | None, child: str | None) -> str | None:
    if parent and child:
        return f"({parent}) and ({child})"
    return parent or child


def convert_v0_variant_selectors(node: Any, inherited: str | None = None) -> Any:
    """Convert conda-build comment selectors to rattler-build v1 conditionals.

    Variant selectors occur on mapping keys and sequence entries. V1 evaluates
    conditionals in lists, so a selector attached to a key is inherited by each
    entry in its value list.
    """
    if isinstance(node, Mapping):
        result = {}
        comments = getattr(getattr(node, "ca", None), "items", {})
        for key, value in node.items():
            condition = _combine_selectors(
                inherited, _selector_from_comment(comments.get(key))
            )
            result[key] = convert_v0_variant_selectors(value, condition)
        return result

    if isinstance(node, Sequence) and not isinstance(node, (str, bytes)):
        result = []
        comments = getattr(getattr(node, "ca", None), "items", {})
        for index, value in enumerate(node):
            condition = _combine_selectors(
                inherited, _selector_from_comment(comments.get(index))
            )
            converted = convert_v0_variant_selectors(value)
            if condition:
                converted = {"if": condition, "then": converted}
            result.append(converted)
        return result

    return copy.deepcopy(node)


def _normalized_package_name(spec: str) -> str | None:
    """Extract a normalized package name from a requirement string."""
    spec = spec.strip()
    if not spec or spec.startswith("${{"):
        return None
    match = re.match(r"[A-Za-z0-9_.-]+", spec)
    return match.group(0).lower().replace("-", "_") if match else None


def _collect_requirement_names(node: Any, names: set[str]) -> None:
    if isinstance(node, str):
        if name := _normalized_package_name(node):
            names.add(name)
        return
    if isinstance(node, Mapping):
        for key, value in node.items():
            if key != "if":
                _collect_requirement_names(value, names)
        return
    if isinstance(node, Sequence) and not isinstance(node, (str, bytes)):
        for value in node:
            _collect_requirement_names(value, names)


def _collect_recipe_requirement_names(node: Any, names: set[str]) -> None:
    """Find requirement sections at the recipe top level and inside tests."""
    if isinstance(node, Mapping):
        for key, value in node.items():
            if key == "requirements":
                _collect_requirement_names(value, names)
            else:
                _collect_recipe_requirement_names(value, names)
    elif isinstance(node, Sequence) and not isinstance(node, (str, bytes)):
        for value in node:
            _collect_recipe_requirement_names(value, names)


def _collect_variant_expressions(node: Any, strings: list[str]) -> None:
    """Collect Jinja expressions and selector conditions, not arbitrary scripts."""
    if isinstance(node, str):
        if "${{" in node:
            strings.append(node)
    elif isinstance(node, Mapping):
        for key, value in node.items():
            if key == "if" and isinstance(value, str):
                strings.append(value)
            else:
                _collect_variant_expressions(value, strings)
    elif isinstance(node, Sequence) and not isinstance(node, (str, bytes)):
        for value in node:
            _collect_variant_expressions(value, strings)


def _used_variant_keys(
    recipe: Mapping[str, Any], variants: Mapping[str, Any]
) -> set[str]:
    """Find variant keys referenced by a recipe's dependencies and expressions."""
    used: set[str] = set()
    _collect_recipe_requirement_names(recipe, used)

    strings: list[str] = []
    _collect_variant_expressions(recipe, strings)
    variant_keys = set(variants) - _SPECIAL_VARIANT_KEYS
    for value in strings:
        for match in _COMPILER_RE.finditer(value):
            language = match.group("language")
            used.update({f"{language}_compiler", f"{language}_compiler_version"})
        for match in _STDLIB_RE.finditer(value):
            language = match.group("language")
            used.update({f"{language}_stdlib", f"{language}_stdlib_version"})
        used.update(
            match.group("name")
            for match in _JINJA_NAME_RE.finditer(value)
            if match.group("name") in variant_keys
        )
    return used


def _prune_zip_keys(groups: Any, used: set[str]) -> list[list[str]]:
    if not groups:
        return []
    if groups and isinstance(groups[0], str):
        groups = [groups]
    result = []
    for group in groups:
        retained = [key for key in group if key in used]
        if len(retained) > 1:
            result.append(retained)
    return result


def get_recipe_variants(
    recipe: Mapping[str, Any],
    package_name: str,
    vinca_conf: Mapping[str, Any],
) -> dict[str, Any]:
    """Return a v1, package-local subset of the repository variant config."""
    variants = copy.deepcopy(vinca_conf.get("_variant_config") or {})
    overrides = copy.deepcopy(
        vinca_conf.get("_pkg_additional_info", {})
        .get(package_name, {})
        .get("variant_overrides", {})
    )
    variants.update(overrides)

    used = _used_variant_keys(recipe, variants) | set(overrides)
    result = {
        key: value
        for key, value in variants.items()
        if key in used and key not in _SPECIAL_VARIANT_KEYS
    }

    if groups := _prune_zip_keys(variants.get("zip_keys"), used):
        result["zip_keys"] = groups

    pin_run_as_build = {
        key: value
        for key, value in (variants.get("pin_run_as_build") or {}).items()
        if key.lower().replace("-", "_") in used
    }
    if pin_run_as_build:
        result["pin_run_as_build"] = pin_run_as_build
    return result
