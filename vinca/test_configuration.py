from vinca.configuration import read_snapshot, read_vinca_yaml


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

    assert config["_patches"]["demo"]["any"] == [str(patches / "demo.patch")]
    assert config["_patches"]["demo"]["linux"] == [str(patches / "demo.unix.patch")]
    assert config["_patches"]["demo"]["osx"] == [str(patches / "demo.unix.patch")]
    assert config["_patches"]["demo"]["win"] == [str(patches / "demo.win.patch")]
    assert config["_tests"]["demo"] == tests / "demo.yaml"
    assert config["_tests"]["dotted"] == tests / "dotted.name.yaml"
    assert config["_test_folders"]["demo"] == tests / "demo"
    assert config["_pkg_additional_info"]["demo"]["build_number"] == 2
    assert config["depmods"] == {"demo": {}}


def test_read_snapshot_merges_additional_packages(tmp_path):
    snapshot = tmp_path / "snapshot.yaml"
    additional = tmp_path / "additional.yaml"
    snapshot.write_text("existing:\n  version: 1\noverridden:\n  version: 1\n")
    additional.write_text("overridden:\n  version: 2\nnew:\n  version: 1\n")

    merged, loaded_additional = read_snapshot(
        {
            "rosdistro_snapshot": str(snapshot),
            "rosdistro_additional_recipes": str(additional),
        }
    )

    assert merged == {
        "existing": {"version": 1},
        "overridden": {"version": 2},
        "new": {"version": 1},
    }
    assert loaded_additional == {
        "overridden": {"version": 2},
        "new": {"version": 1},
    }
