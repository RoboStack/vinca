#!/usr/bin/env python3
"""Sort top-level keys in YAML mapping files using ruamel.yaml (preserves comments).

Usage:
  vinca-sort-yaml-keys FILE [FILE ...]
  vinca-sort-yaml-keys --check FILE [FILE ...]
"""

import shutil
import sys
import tempfile
from io import StringIO
from pathlib import Path

from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedMap


def sort_mapping_keys(path: Path) -> bool:
    """Sort top-level keys of a YAML mapping file in place. Returns True if changed."""
    yaml = YAML()
    yaml.preserve_quotes = True
    yaml.width = 4096

    text_before = path.read_text()
    data = yaml.load(text_before)
    if not hasattr(data, "keys"):
        return False

    sorted_keys = sorted(data.keys(), key=str.casefold)
    if list(data.keys()) == sorted_keys:
        return False

    new_data = CommentedMap()
    if data.ca and data.ca.comment:
        new_data.ca.comment = data.ca.comment
    for key in sorted_keys:
        new_data[key] = data[key]
        if key in data.ca.items:
            new_data.ca.items[key] = data.ca.items[key]

    buf = StringIO()
    yaml.dump(new_data, buf)
    text_after = buf.getvalue()
    if text_after != text_before:
        path.write_text(text_after)
        return True
    return False


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+", type=Path, help="YAML files to sort")
    parser.add_argument(
        "--check", action="store_true", help="Check only, exit 1 if unsorted"
    )
    args = parser.parse_args()

    any_changed = False
    for path in args.files:
        if not path.exists():
            print(f"ERROR: {path} not found", file=sys.stderr)
            sys.exit(1)
        if args.check:
            with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as tf:
                tmp = Path(tf.name)
            shutil.copy2(path, tmp)
            changed = sort_mapping_keys(tmp)
            tmp.unlink(missing_ok=True)
        else:
            changed = sort_mapping_keys(path)

        status = ("UNSORTED" if args.check else "SORTED") if changed else "OK"
        print(f"{status}: {path}")
        any_changed = any_changed or changed

    if args.check and any_changed:
        sys.exit(1)


if __name__ == "__main__":
    main()
