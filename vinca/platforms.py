"""Conda platform detection utilities."""

from __future__ import annotations

import platform
import sys

_PLATFORM_BY_SYSTEM_AND_MACHINE = {
    ("linux", "aarch64"): "linux-aarch64",
    ("linux", "x86_64"): "linux-64",
    ("darwin", "arm64"): "osx-arm64",
    ("darwin", "x86_64"): "osx-64",
    ("win32", "AMD64"): "win-64",
    ("win32", "x86_64"): "win-64",
}


def get_conda_subdir(selected_platform: str | None = None) -> str:
    """Return an explicit target platform or detect the host conda subdir."""
    if selected_platform:
        return selected_platform

    system = sys.platform
    if system.startswith("linux"):
        system = "linux"

    machine = platform.machine()
    try:
        return _PLATFORM_BY_SYSTEM_AND_MACHINE[system, machine]
    except KeyError as error:
        raise RuntimeError(
            f"Unsupported platform: system={system!r}, machine={machine!r}"
        ) from error
