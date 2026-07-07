#!/usr/bin/env python3
"""Sort list items in vinca.yaml (line-based, preserves comments and formatting).

Sorts plain `  - item` entries alphabetically within each target list key.
Conditional blocks (`- if: ... then: [...]`) stay at the end; their inner
`then:` lists are also sorted.

Usage:
  vinca-sort-vinca-lists [FILE]
  vinca-sort-vinca-lists --check [FILE]
"""

import re
import shutil
import sys
import tempfile
from pathlib import Path

# Top-level keys whose list items should be sorted
LISTS_TO_SORT = {
    "packages_select_by_deps",
    "packages_skip_by_deps",
    "packages_remove_from_deps",
}

# Regex for a simple list item line: "  - value" with optional inline comment
RE_SIMPLE_ITEM = re.compile(r"^  - (\S.*)$")
# Regex for the start of a conditional block: "  - if: ..."
RE_IF_BLOCK = re.compile(r"^  - if:")
# Regex for a then-list item inside a conditional block: "      - value"
RE_THEN_ITEM = re.compile(r"^      - (\S.*)$")
# Regex for a top-level key
RE_TOP_KEY = re.compile(r"^(\S+):")


def _sort_key(line: str) -> str:
    """Extract sortable value from a list item line (lowercase, ignore comments)."""
    m = RE_SIMPLE_ITEM.match(line) or RE_THEN_ITEM.match(line)
    if m:
        val = m.group(1).split("#")[0].strip()
        return val.casefold()
    return line.casefold()


def sort_vinca_lists(path: Path) -> bool:
    """Sort list items in vinca.yaml in place. Returns True if changed."""
    text = path.read_text()
    lines = text.splitlines(keepends=True)

    result = []
    i = 0
    changed = False

    while i < len(lines):
        line = lines[i]

        # Check if this line starts a target list key
        m = RE_TOP_KEY.match(line)
        if m and m.group(1) in LISTS_TO_SORT:
            result.append(line)
            i += 1

            # Collect all content belonging to this list
            simple_items = []  # plain "  - value" lines
            if_blocks = []  # multi-line conditional blocks
            current_if_block = None

            while i < len(lines):
                line = lines[i]

                # Next top-level key or end of list
                if RE_TOP_KEY.match(line):
                    break

                # Blank line
                if line.strip() == "":
                    if current_if_block is not None:
                        current_if_block.append(line)
                    # Skip blank lines between simple items (sorting removes grouping)
                    i += 1
                    continue

                # Comment-only line at list level (e.g. "  # These packages...")
                if line.startswith("  #") and not line.startswith("      "):
                    # Only finalize if current block has actual if/then content
                    if current_if_block is not None and any(
                        RE_IF_BLOCK.match(bl) for bl in current_if_block
                    ):
                        if_blocks.append(current_if_block)
                        current_if_block = [line]
                    elif current_if_block is not None:
                        current_if_block.append(line)
                    else:
                        current_if_block = [line]
                    i += 1
                    continue

                # Start of conditional block: "  - if: ..."
                if RE_IF_BLOCK.match(line):
                    if current_if_block is None:
                        current_if_block = []
                    current_if_block.append(line)
                    i += 1
                    continue

                # Inside conditional block (then:, items, etc.)
                if current_if_block is not None:
                    current_if_block.append(line)
                    i += 1
                    continue

                # Simple list item: "  - value"
                if RE_SIMPLE_ITEM.match(line):
                    simple_items.append(line)
                    i += 1
                    continue

                # Something unexpected; pass through
                result.append(line)
                i += 1
                continue

            # Finalize any pending if-block
            if current_if_block is not None:
                if_blocks.append(current_if_block)

            # Sort simple items
            sorted_simple = sorted(simple_items, key=_sort_key)
            if sorted_simple != simple_items:
                changed = True

            # Sort then-lists inside each if-block
            for block in if_blocks:
                then_items = []
                then_indices = []
                blank_indices = []
                in_then = False
                for bi, bline in enumerate(block):
                    if bline.strip() == "then:":
                        in_then = True
                        continue
                    if in_then:
                        if RE_THEN_ITEM.match(bline):
                            then_items.append(bline)
                            then_indices.append(bi)
                        elif bline.strip() == "":
                            blank_indices.append(bi)

                sorted_then = sorted(then_items, key=_sort_key)
                if sorted_then != then_items:
                    changed = True
                    for bi, bline in zip(then_indices, sorted_then):
                        block[bi] = bline

                # Remove blank lines between then-items (sorting removes grouping)
                for bi in reversed(blank_indices):
                    # Only remove if between then-items (not trailing)
                    if any(ti < bi for ti in then_indices) and any(
                        ti > bi for ti in then_indices
                    ):
                        block.pop(bi)
                        changed = True

            # Write: sorted simple items, then if-blocks
            for item in sorted_simple:
                result.append(item)

            for block in if_blocks:
                # Strip trailing blank lines from block
                while block and block[-1].strip() == "":
                    block.pop()
                # Ensure single blank line before block
                if result and result[-1].strip() != "":
                    result.append("\n")
                for bline in block:
                    result.append(bline)
            # Trailing blank line after all blocks
            if if_blocks:
                result.append("\n")

            # Ensure blank line after the list if next line is a key
            if result and result[-1].strip() != "":
                result.append("\n")

            continue

        # Non-target line, pass through
        result.append(line)
        i += 1

    new_text = "".join(result)
    if new_text != text:
        changed = True
        path.write_text(new_text)
    return changed


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", nargs="?", type=Path, default=Path("vinca.yaml"))
    parser.add_argument(
        "--check", action="store_true", help="Check only, exit 1 if unsorted"
    )
    args = parser.parse_args()

    if not args.file.exists():
        print(f"ERROR: {args.file} not found", file=sys.stderr)
        sys.exit(1)

    if args.check:
        with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as tf:
            tmp = Path(tf.name)
        shutil.copy2(args.file, tmp)
        changed = sort_vinca_lists(tmp)
        tmp.unlink(missing_ok=True)
    else:
        changed = sort_vinca_lists(args.file)

    status = ("UNSORTED" if args.check else "SORTED") if changed else "OK"
    print(f"{status}: {args.file}")
    if args.check and changed:
        sys.exit(1)


if __name__ == "__main__":
    main()
