"""Conda platform detection utilities."""

from __future__ import annotations

import platform
import sys

_PLATFORM_BY_SYSTEM_AND_MACHINE = {
    ("linux", "aarch64"): "linux-aarch64",
    ("linux", "x86_64"): "linux-64",
    ("darwin", "arm64"): "osx-arm64",
}

# Systems whose non-x86_64 machines still build against the 64-bit subdir, either
# because the toolchain is universal or because the runner emulates it.
_DEFAULT_PLATFORM_BY_SYSTEM = {
    "darwin": "osx-64",
    "win32": "win-64",
}


def get_conda_subdir(selected_platform: str | None = None) -> str:
    """Return an explicit target platform or detect the host conda subdir."""
    if selected_platform:
        return selected_platform

    system = sys.platform
    if system.startswith("linux"):
        system = "linux"

    machine = platform.machine()
    subdir = _PLATFORM_BY_SYSTEM_AND_MACHINE.get(
        (system, machine)
    ) or _DEFAULT_PLATFORM_BY_SYSTEM.get(system)
    if subdir is None:
        raise RuntimeError(
            f"Unsupported platform: system={system!r}, machine={machine!r}"
        )
    return subdir
