"""A lightweight implementation of the CFEP-9 variant algebra.

This is adapted from ``conda_smithy.variant_algebra`` (BSD-3-Clause).  Keeping
the small algebra here avoids making the recipe generator depend on all of
conda-build and conda-smithy, while matching the semantics conda-forge uses for
normal migrations, zip keys, pin_run_as_build, key_add, and key_remove.
"""

from __future__ import annotations

from copy import deepcopy
import re

from packaging.version import InvalidVersion, Version
from ruamel.yaml.comments import CommentedSeq


_SELECTOR_RE = re.compile(r"#\s*\[(.+)]")
_PLATFORMS = {
    "linux-64",
    "linux-aarch64",
    "osx-64",
    "osx-arm64",
    "win-64",
}


def _ensure_list(value):
    if isinstance(value, list):
        return value
    return [value]


def _version_order(value, ordering=None):
    if ordering is not None:
        return (2, ordering.index(value))
    normalized = str(value).replace(" ", ".").replace("*", "1")
    try:
        return (1, Version(normalized))
    except InvalidVersion:
        return (0, normalized)


def _copy_sequence_comments(values, *sources):
    """Build a round-trip sequence, retaining selector comments where possible."""
    result = CommentedSeq(deepcopy(list(values)))
    used = [set() for _ in sources]
    for output_index, value in enumerate(values):
        for source_number, source in enumerate(sources):
            if not isinstance(source, CommentedSeq):
                continue
            for source_index, candidate in enumerate(source):
                if source_index in used[source_number] or candidate != value:
                    continue
                used[source_number].add(source_index)
                if source_index in source.ca.items:
                    result.ca.items[output_index] = deepcopy(
                        source.ca.items[source_index]
                    )
                break
            else:
                continue
            break
    return result


def _comment_selector(comment):
    if comment is None:
        return None
    match = _SELECTOR_RE.search(comment.value)
    return match.group(1) if match else None


def _sequence_selector(mapping, key, index):
    key_comment = mapping.ca.items.get(key, [None, None, None])[2]
    key_selector = _comment_selector(key_comment)
    value = mapping[key]
    item_comment = value.ca.items.get(index, [None])[0]
    item_selector = _comment_selector(item_comment)
    if key_selector and item_selector:
        if key_selector == item_selector:
            return key_selector
        return f"({key_selector}) and ({item_selector})"
    return key_selector or item_selector


def _selector_platforms(selector):
    if selector is None:
        return set(_PLATFORMS)
    platforms = set(_PLATFORMS)
    tokens = set(re.findall(r"[A-Za-z0-9_]+", selector))
    if "unix" in tokens:
        platforms &= {item for item in _PLATFORMS if not item.startswith("win-")}
    os_scopes = []
    def is_positive_os(*names):
        return any(name in tokens for name in names) and not any(
            re.search(rf"\bnot\s+{name}\b", selector) for name in names
        )

    if is_positive_os("linux", "linux64"):
        os_scopes.append({item for item in _PLATFORMS if item.startswith("linux-")})
    if is_positive_os("osx"):
        os_scopes.append({item for item in _PLATFORMS if item.startswith("osx-")})
    if is_positive_os("win", "win64"):
        os_scopes.append({item for item in _PLATFORMS if item.startswith("win-")})
    if os_scopes:
        platforms &= set().union(*os_scopes)
    arch_scopes = []
    if "x86_64" in tokens:
        arch_scopes.append({"linux-64", "osx-64", "win-64"})
    if "aarch64" in tokens or "arm64" in tokens:
        arch_scopes.append({"linux-aarch64", "osx-arm64"})
    if arch_scopes:
        platforms &= set().union(*arch_scopes)
    for os_name, prefix in (("linux", "linux-"), ("osx", "osx-"), ("win", "win-")):
        if re.search(rf"\bnot\s+{os_name}\b", selector):
            platforms -= {item for item in _PLATFORMS if item.startswith(prefix)}
    return platforms


def _platform_selector(platforms):
    if platforms == {"linux-64", "linux-aarch64"}:
        return "linux"
    if platforms == {"osx-64", "osx-arm64"}:
        return "osx"
    if platforms == {"linux-64", "linux-aarch64", "osx-64", "osx-arm64"}:
        return "unix"
    expressions = {
        "linux-64": "linux and x86_64",
        "linux-aarch64": "linux and aarch64",
        "osx-64": "osx and x86_64",
        "osx-arm64": "osx and arm64",
        "win-64": "win64",
    }
    return " or ".join(expressions[item] for item in sorted(platforms))


def _selector_aware_replace(left_mapping, right_mapping, key):
    """Replace only selector scopes present in the migration's sequence."""
    left = left_mapping[key]
    right = right_mapping[key]
    right_selectors = [
        _sequence_selector(right_mapping, key, index) for index in range(len(right))
    ]
    affected = set().union(
        *(_selector_platforms(selector) for selector in right_selectors)
    )
    values = []
    selectors = []
    sources = []
    for index, value in enumerate(left):
        selector = _sequence_selector(left_mapping, key, index)
        scope = _selector_platforms(selector)
        remaining = scope - affected
        if not remaining:
            continue
        values.append(value)
        sources.append((left, index))
        selectors.append(None if remaining == scope else _platform_selector(remaining))
    for index, value in enumerate(right):
        values.append(value)
        sources.append((right, index))
        selectors.append(right_selectors[index])

    result = CommentedSeq(deepcopy(values))
    for output_index, ((source, source_index), selector) in enumerate(
        zip(sources, selectors)
    ):
        if source_index in source.ca.items:
            result.ca.items[output_index] = deepcopy(source.ca.items[source_index])
        if (
            selector
            and _comment_selector(result.ca.items.get(output_index, [None])[0])
            != selector
        ):
            result.yaml_add_eol_comment(f"[{selector}]", output_index)
    return result


def _has_sequence_selectors(mapping, key):
    value = mapping[key]
    return isinstance(value, CommentedSeq) and any(
        _sequence_selector(mapping, key, index) is not None
        for index in range(len(value))
    )


def _key_add(left, right, ordering=None):
    output = []
    common_length = min(len(left), len(right))
    if ordering is None:
        for index in range(common_length):
            if _version_order(left[index]) < _version_order(right[index]):
                output.append(right[index])
            else:
                output.append(left[index])
        for values in (left, right):
            output.extend(values[common_length:])
    else:
        ordinals = sorted(
            {_version_order(value, ordering)[1] for value in list(left) + list(right)}
        )
        longer = max(len(left), len(right))
        if len(ordinals) < longer:
            if len(ordinals) == 1:
                output = [left[0]] * longer
            else:
                raise ValueError(
                    "ambiguous merge due to duplicate values and non-None ordering"
                )
        else:
            output = [ordering[index] for index in ordinals[-longer:]]
    return _copy_sequence_comments(output, left, right)


def _set_union(left, right, ordering=None):
    values = set(left) | set(right)
    return sorted(values, key=lambda value: _version_order(value, ordering))


def _key_add_operation(left, right):
    primary_key = right["__migrator"]["primary_key"]
    additional_zip_keys = right["__migrator"].get("additional_zip_keys", [])
    ordering = right["__migrator"].get("ordering", {})
    if primary_key not in right:
        return left
    if primary_key not in left:
        raise RuntimeError(f"key_add primary key {primary_key!r} is missing")

    result = deepcopy(left)
    new_zip_keys = set()
    primary_group = [primary_key]
    for chunk in result.get("zip_keys", []):
        if primary_key in chunk:
            primary_group = chunk

    if additional_zip_keys:
        for chunk in result.get("zip_keys", []):
            if primary_key in chunk:
                for key in additional_zip_keys:
                    if key not in chunk:
                        chunk.append(key)
                        new_zip_keys.add(key)
                primary_group = list(chunk)
                break
        else:
            primary_group = [primary_key] + list(additional_zip_keys)
            result.setdefault("zip_keys", CommentedSeq()).append(primary_group)
            new_zip_keys.update(additional_zip_keys)

    for primary_index in range(len(right[primary_key])):
        current = deepcopy(result)
        existing_count = len(current[primary_key])
        for key in new_zip_keys:
            if len(current[key]) != 1:
                raise ValueError(
                    f"Cannot broadcast non-unit-length {key} to {existing_count} entries"
                )
            current[key] = [current[key][0]] * existing_count
        for key in primary_group:
            if key not in right:
                raise ValueError(f"Required zip_key {key} not specified in migration")

        existing = list(zip(*(current[key] for key in primary_group)))
        new = tuple(right[key][primary_index] for key in primary_group)
        if new in existing:
            continue
        primary_position = primary_group.index(primary_key)
        merged = sorted(
            _set_union(existing, [new]),
            key=lambda item: _version_order(
                item[primary_position], ordering.get(primary_key)
            ),
        )
        columns = list(zip(*merged))
        for index, key in enumerate(primary_group):
            result[key] = _copy_sequence_comments(
                list(columns[index]), left.get(key, []), right.get(key, [])
            )

    extra_ordering = set(ordering) - new_zip_keys - {primary_key}
    for key in extra_ordering:
        result[key] = _key_add(left[key], right[key], ordering[key])
    return result


def _key_remove_operation(left, right):
    primary_key = right["__migrator"]["primary_key"]
    ordering = right["__migrator"].get("ordering", {})
    if primary_key not in right or primary_key not in left:
        return left
    if len(right[primary_key]) != 1:
        raise ValueError("key_remove migrations must contain one primary value")
    removed = right[primary_key][0]
    if removed not in left[primary_key]:
        return left

    result = deepcopy(left)
    values = _set_union(left[primary_key], [], ordering.get(primary_key))
    values.remove(removed)
    positions = [left[primary_key].index(value) for value in values]
    result[primary_key] = _copy_sequence_comments(values, left[primary_key])
    for chunk in left.get("zip_keys", []):
        if primary_key in chunk:
            for key in chunk:
                if key != primary_key:
                    selected = [left[key][index] for index in positions]
                    result[key] = _copy_sequence_comments(selected, left[key])
    return result


def _merge_zip_keys(left, right):
    output = []
    left_sets = {frozenset(chunk) for chunk in left}
    right_sets = {frozenset(chunk) for chunk in right}
    for right_set in sorted(right_sets, key=lambda item: -len(item)):
        for left_set in sorted(left_sets, key=lambda item: -len(item)):
            if left_set.issubset(right_set):
                left_sets.remove(left_set)
                right_sets.remove(right_set)
                output.append(right_set)
                break
    output.extend(left_sets)
    output.extend(right_sets)
    return CommentedSeq(
        sorted(
            [sorted(item) for item in output], key=lambda item: (len(item), str(item))
        )
    )


def variant_add(left, right):
    """Combine two variant mappings with conda-forge's CFEP-9 semantics."""
    operation = (right.get("__migrator") or {}).get("operation")
    if operation == "key_add":
        return _key_add_operation(left, right)
    if operation == "key_remove":
        return _key_remove_operation(left, right)
    if operation:
        raise ValueError(f"Unknown migration operation: {operation}")

    ordering = (right.get("__migrator") or {}).get("ordering", {})
    primary_keys = _ensure_list((right.get("__migrator") or {}).get("primary_key", []))
    result = deepcopy(left)
    right_keys = {key for key in right if key != "__migrator" and key != "migrator_ts"}

    if "pin_run_as_build" in right_keys and "pin_run_as_build" in left:
        merged = deepcopy(left["pin_run_as_build"])
        merged.update(deepcopy(right["pin_run_as_build"]))
        result["pin_run_as_build"] = merged
        right_keys.remove("pin_run_as_build")

    if "zip_keys" in right_keys and "zip_keys" in left:
        result["zip_keys"] = _merge_zip_keys(left["zip_keys"], right["zip_keys"])
        right_keys.remove("zip_keys")

    handled = set()
    zip_groups = list(left.get("zip_keys", [])) + list(right.get("zip_keys", []))
    for primary_key in primary_keys:
        if primary_key not in right or primary_key not in left:
            continue
        primary_left = _ensure_list(left[primary_key])
        primary_right = _ensure_list(right[primary_key])
        if _has_sequence_selectors(right, primary_key):
            merged = _selector_aware_replace(left, right, primary_key)
        else:
            merged = _key_add(primary_left, primary_right, ordering.get(primary_key))
        result[primary_key] = merged
        group = next((group for group in zip_groups if primary_key in group), [])
        if not group:
            continue
        source_primary = primary_left + primary_right
        chosen = []
        for value in merged:
            chosen.append(
                next(
                    index
                    for index, candidate in enumerate(source_primary)
                    if candidate == value and index not in chosen
                )
            )
        for key in set(group) - {primary_key}:
            values = _ensure_list(left[key]) + _ensure_list(right[key])
            selected = [value for index, value in enumerate(values) if index in chosen]
            result[key] = _copy_sequence_comments(selected, left[key], right[key])
        handled.update(group)

    for key in right_keys - handled:
        if key in left:
            if _has_sequence_selectors(right, key):
                result[key] = _selector_aware_replace(left, right, key)
            else:
                result[key] = _key_add(
                    _ensure_list(left[key]),
                    _ensure_list(right[key]),
                    ordering.get(key),
                )
        else:
            result[key] = deepcopy(right[key])
    return result
