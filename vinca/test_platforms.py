import pytest

from vinca import platforms


def test_explicit_conda_subdir_wins(monkeypatch):
    monkeypatch.setattr(platforms.sys, "platform", "unsupported")

    assert platforms.get_conda_subdir("emscripten-wasm32") == "emscripten-wasm32"


@pytest.mark.parametrize(
    ("system", "machine", "expected"),
    [
        ("linux", "x86_64", "linux-64"),
        ("linux", "aarch64", "linux-aarch64"),
        ("darwin", "arm64", "osx-arm64"),
        ("darwin", "x86_64", "osx-64"),
        ("win32", "AMD64", "win-64"),
    ],
)
def test_detect_conda_subdir(monkeypatch, system, machine, expected):
    monkeypatch.setattr(platforms.sys, "platform", system)
    monkeypatch.setattr(platforms.platform, "machine", lambda: machine)

    assert platforms.get_conda_subdir() == expected


def test_unknown_conda_subdir_has_actionable_error(monkeypatch):
    monkeypatch.setattr(platforms.sys, "platform", "plan9")
    monkeypatch.setattr(platforms.platform, "machine", lambda: "mips")

    with pytest.raises(RuntimeError, match="system='plan9'.*machine='mips'"):
        platforms.get_conda_subdir()
