import pytest
from pydantic import ValidationError

from vinca.configuration import read_snapshot, read_vinca_yaml
from vinca.schemas import DepMods, SnapshotEntry, VariantsMode, VincaYaml


def test_read_vinca_yaml_discovers_companion_files(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    patches = tmp_path / "patches"
    patches.mkdir()
    (patches / "demo.patch").write_text("generic")
    (patches / "demo.unix.patch").write_text("unix")
    (patches / "demo.win.patch").write_text("windows")
    (patches / "dependencies.yaml").write_text("demo: {}\n")

    tests = tmp_path / "tests"
    tests.mkdir()
    (tests / "demo.yaml").write_text("tests: []\n")
    (tests / "dotted.name.yaml").write_text("tests: []\n")
    (tests / "demo").mkdir()
    (tmp_path / "pkg_additional_info.yaml").write_text("demo:\n  build_number: 2\n")
    (tmp_path / "vinca.yaml").write_text(
        "ros_distro: humble\nconda_index: []\npatch_dir: patches\nskip_testing: false\n"
    )

    config = read_vinca_yaml(tmp_path / "vinca.yaml", "linux-64")

    assert config.patches["demo"]["any"] == [str(patches / "demo.patch")]
    assert config.patches["demo"]["linux"] == [str(patches / "demo.unix.patch")]
    assert config.patches["demo"]["osx"] == [str(patches / "demo.unix.patch")]
    assert config.patches["demo"]["win"] == [str(patches / "demo.win.patch")]
    assert config.tests["demo"] == tests / "demo.yaml"
    assert config.tests["dotted"] == tests / "dotted.name.yaml"
    assert config.test_folders["demo"] == tests / "demo"
    assert config.pkg_additional_info["demo"].build_number == 2
    assert config.depmods == {"demo": DepMods()}
    assert config.variants_mode is VariantsMode.GLOBAL
    assert config.variant_config == {}


def test_read_vinca_yaml_loads_local_variant_config(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "patches").mkdir()
    (tmp_path / "vinca.yaml").write_text(
        "ros_distro: humble\nconda_index: []\npatch_dir: patches\nvariants_mode: local\n"
    )
    (tmp_path / "conda_build_config.yaml").write_text(
        "c_compiler:\n"
        "  - gcc  # [linux]\n"
        "  - clang  # [osx]\n"
        "c_compiler_version:  # [unix]\n"
        "  - 14  # [linux]\n"
        "  - 19  # [osx]\n"
    )

    config = read_vinca_yaml(tmp_path / "vinca.yaml", "linux-64")

    assert config.variants_mode is VariantsMode.LOCAL
    assert config.variant_config == {
        "c_compiler": [
            {"if": "linux", "then": "gcc"},
            {"if": "osx", "then": "clang"},
        ],
        "c_compiler_version": [
            {"if": "(unix) and (linux)", "then": 14},
            {"if": "(unix) and (osx)", "then": 19},
        ],
    }


def test_read_vinca_yaml_rejects_invalid_variants_mode(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "vinca.yaml").write_text(
        "ros_distro: humble\nconda_index: []\npatch_dir: patches\nvariants_mode: other\n"
    )

    with pytest.raises(ValidationError, match=r"(?s)vinca\.yaml.*variants_mode"):
        read_vinca_yaml(tmp_path / "vinca.yaml", "linux-64")


def test_read_vinca_yaml_requires_config_for_local_pinning(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "patches").mkdir()
    (tmp_path / "vinca.yaml").write_text(
        "ros_distro: humble\nconda_index: []\npatch_dir: patches\nvariants_mode: local\n"
    )

    with pytest.raises(FileNotFoundError, match="requires conda_build_config.yaml"):
        read_vinca_yaml(tmp_path / "vinca.yaml", "linux-64")


def test_read_snapshot_merges_additional_packages(tmp_path):
    snapshot = tmp_path / "snapshot.yaml"
    additional = tmp_path / "additional.yaml"
    snapshot.write_text(
        "existing:\n  url: https://github.com/x/existing.git\n  version: 1\n"
        "overridden:\n  url: https://github.com/x/overridden.git\n  version: 1\n"
    )
    additional.write_text(
        "overridden:\n  url: https://github.com/x/overridden.git\n  version: 2\n"
        "new:\n  url: https://github.com/x/new.git\n  version: 1\n"
    )

    merged, loaded_additional = read_snapshot(
        VincaYaml(
            ros_distro="humble",
            rosdistro_snapshot=str(snapshot),
            rosdistro_additional_recipes=str(additional),
        )
    )

    assert merged == {
        "existing": SnapshotEntry(url="https://github.com/x/existing.git", version="1"),
        "overridden": SnapshotEntry(
            url="https://github.com/x/overridden.git", version="2"
        ),
        "new": SnapshotEntry(url="https://github.com/x/new.git", version="1"),
    }
    assert loaded_additional == {
        "overridden": SnapshotEntry(
            url="https://github.com/x/overridden.git", version="2"
        ),
        "new": SnapshotEntry(url="https://github.com/x/new.git", version="1"),
    }
