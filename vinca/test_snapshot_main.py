import sys

import yaml

from vinca import snapshot


class FakeDistro:
    """Two released packages; the release repository of `gone` was removed."""

    def __init__(self, name, distribution_cache=None):
        pass

    def get_package_names(self):
        return ["gone", "kept"]

    def get_released_repo(self, pkg):
        return f"https://github.com/ros2-gbp/{pkg}-release.git", f"release/{pkg}", "tag"

    def get_version(self, pkg):
        return "1.0.0"

    def get_release_package_xml(self, pkg):
        return None if pkg == "gone" else "<package/>"

    def get_direct_depends(self, pkg):
        if pkg == "gone":
            raise AssertionError("dependencies of a package without package.xml read")
        return {"rclcpp"}

    def get_repository_url(self, pkg):
        return None


def test_snapshot_skips_package_without_package_xml(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(snapshot, "load_distribution_cache", lambda *args: None)
    monkeypatch.setattr(snapshot, "Distro", FakeDistro)
    output = tmp_path / "rosdistro_snapshot.yaml"
    monkeypatch.setattr(sys, "argv", ["vinca-snapshot", "-o", str(output)])

    snapshot.main()

    written = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert set(written) == {"kept"}
    assert written["kept"]["dependencies"] == ["rclcpp"]
    assert "Package 'gone' has no package.xml" in capsys.readouterr().out
