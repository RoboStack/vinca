"""Tests for the vinca.yaml list sorter."""

from vinca.sort_vinca_lists import sort_vinca_lists

BASE = """packages_select_by_deps:
  - alpha
  - charlie
  - bravo
"""


def test_sorts_simple_top_level_items(tmp_path):
    path = tmp_path / "vinca.yaml"
    path.write_text(BASE)

    changed = sort_vinca_lists(path)

    assert changed is True
    items = [
        line.strip()[2:]
        for line in path.read_text().splitlines()
        if line.startswith("  - ")
    ]
    assert items == ["alpha", "bravo", "charlie"]


def test_leaves_already_sorted_file_unchanged(tmp_path):
    path = tmp_path / "vinca.yaml"
    sorted_content = "packages_select_by_deps:\n  - alpha\n  - bravo\n  - charlie\n\n"
    path.write_text(sorted_content)

    changed = sort_vinca_lists(path)

    assert changed is False
    assert path.read_text() == sorted_content


def test_adjacent_if_blocks_without_separator_stay_isolated(tmp_path):
    # Regression test: two "- if:" blocks back-to-back with no blank line or
    # comment between them used to be parsed as a single block, so sorting
    # pooled both blocks' then-items together and could redistribute an item
    # from one block's platform condition into the other's.
    content = """packages_select_by_deps:
  - if: not wasm32 and not win
    then:
      - web_video_server
      - webots_ros2
      - yasmin
      - yasmin_ros
  - if: linux and not aarch64
    then:
      - somepkg
      - zed_msgs

patch_dir: patch
"""
    path = tmp_path / "vinca.yaml"
    path.write_text(content)

    sort_vinca_lists(path)

    result = path.read_text()
    first_block, second_block = result.split("- if: not wasm32 and not win")[1].split(
        "- if: linux and not aarch64"
    )

    assert "webots_ros2" in first_block
    assert "yasmin_ros" in first_block
    assert "webots_ros2" not in second_block
    assert "somepkg" in second_block
    assert "zed_msgs" in second_block


def test_adjacent_if_blocks_separated_by_comment_stay_isolated(tmp_path):
    # The comment-separator workaround must keep working alongside the fix.
    content = """packages_select_by_deps:
  - if: not wasm32 and not win
    then:
      - web_video_server
      - yasmin
      - yasmin_ros

  # webots_ros2 is linux-only
  - if: linux and not aarch64
    then:
      - webots_ros2
      - zed_msgs

patch_dir: patch
"""
    path = tmp_path / "vinca.yaml"
    path.write_text(content)

    sort_vinca_lists(path)

    first_block, second_block = path.read_text().split("- if: linux and not aarch64")
    assert "- webots_ros2" not in first_block
    assert "- webots_ros2" in second_block
