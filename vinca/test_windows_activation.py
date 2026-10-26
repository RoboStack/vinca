"""Real Git Bash + conda regression test (see tests/windows/README.md)."""

import json
import os
import subprocess
from pathlib import Path

import pytest

from vinca.template import (
    generate_activation_scripts_for_recipe,
    generate_build_script_for_recipe,
)

pytestmark = pytest.mark.skipif(os.name != "nt", reason="requires Windows Git Bash")

ROS_VARIABLES = (
    "ROS_DISTRO",
    "ROS_VERSION",
    "ROS_PYTHON_VERSION",
    "ROS_OS_OVERRIDE",
    "ROS_ETC_DIR",
    "AMENT_PREFIX_PATH",
    "AMENT_PYTHON_EXECUTABLE",
    "PYTHONPATH",
    "QT_PLUGIN_PATH",
    "CMAKE_PREFIX_PATH",
)


@pytest.fixture(scope="module")
def activation_recipe(tmp_path_factory):
    if not os.environ.get("VINCA_WINDOWS_ROS_PREFIX"):
        pytest.skip("set VINCA_WINDOWS_ROS_PREFIX to a disposable Windows ROS env")
    recipe = tmp_path_factory.mktemp("activation_recipe")
    generate_activation_scripts_for_recipe(recipe)
    generate_build_script_for_recipe(
        "bld_catkin.bat", recipe / "bld_catkin.bat", "ros-jazzy"
    )
    return recipe


@pytest.mark.parametrize(
    "path_operation", ["prepend-non-duplicate", "append-non-duplicate", "set"]
)
def test_conda_git_bash_activation(tmp_path, path_operation, activation_recipe):
    prefix = Path(os.environ["VINCA_WINDOWS_ROS_PREFIX"]).resolve()
    bash = Path(os.environ["ProgramFiles"]) / "Git/bin/bash.exe"
    assert bash.is_file(), f"Git for Windows Bash is required: {bash}"
    conda = prefix / "Scripts/conda.exe"
    assert conda.is_file(), f"Install tests/windows/environment.yml in {prefix}"
    assert (prefix / "Library/_local_setup_util.py").is_file()

    recipe = activation_recipe
    build = recipe / "bld_catkin.bat"
    # Run the actual installation block, without rebuilding ros_workspace.
    install_block = build.read_text().split(
        'if "%PKG_NAME%" == "ros-jazzy-ros-workspace" (', 1
    )[1]
    install = recipe / "install.bat"
    install.write_text(
        '@echo off\nif "%PKG_NAME%" == "ros-jazzy-ros-workspace" (' + install_block
    )

    env = {
        key: value
        for key, value in os.environ.items()
        if not key.upper().startswith(("CONDA", "_CE_", "ROS_", "AMENT_"))
        and key.upper()
        not in {
            "PYTHONHOME",
            "PYTHONPATH",
            "QT_PLUGIN_PATH",
            "SYS_PREFIX",
            "CMAKE_PREFIX_PATH",
        }
    }
    env.update(
        PREFIX=str(prefix),
        RECIPE_DIR=str(recipe),
        PKG_NAME="ros-jazzy-ros-workspace",
        CONDA_AUTO_ACTIVATE_BASE="false",
        CONDA_NO_PLUGINS="true",
    )

    # Restore any pre-existing hooks, including when an assertion fails.
    hooks = [
        prefix / f"etc/conda/{action}.d/ros-jazzy-ros-workspace_{action}.{ext}"
        for action in ("activate", "deactivate")
        for ext in ("bat", "ps1", "sh")
    ]
    originals = {path: path.read_bytes() if path.exists() else None for path in hooks}
    # Exercise both prepend/append separators and paths with spaces using the
    # real ament helper, rather than a Python executable that mocks its output.
    package_dir = prefix / "Library/share/vinca_activation_probe"
    marker = (
        prefix
        / "Library/share/ament_index/resource_index/packages/vinca_activation_probe"
    )
    assert not package_dir.exists() and not marker.exists()
    extra_path = tmp_path / "hook paths with spaces"
    extra_path.mkdir()
    (extra_path / "vinca_probe.py").write_text("VALUE = 42\n")
    path_value = str(extra_path)
    if path_operation == "set":
        # Keep Git's utilities available for conda itself, but deliberately
        # replace the inherited PATH with a Windows list containing spaces.
        path_value += ";" + str(bash.parent.parent / "usr/bin")
    try:
        package_dir.mkdir()
        marker.touch()
        (package_dir / "package.dsv").write_text(
            f"prepend-non-duplicate;PYTHONPATH;{extra_path}\n"
            f"{path_operation};PATH;{path_value}\n"
            f"append-non-duplicate;CMAKE_PREFIX_PATH;{extra_path}\n"
        )
        result = subprocess.run(
            ["cmd.exe", "/d", "/c", str(install)],
            env=env,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        assert all(path.is_file() for path in hooks)

        probe = tmp_path / "probe.py"
        probe.write_text(
            "import json, os, sys\n"
            "from pathlib import Path\n"
            f"keys = {(*ROS_VARIABLES, 'PATH')!r}\n"
            "data = {key: os.environ[key] for key in keys if key in os.environ}\n"
            "if sys.argv[2] == 'active':\n"
            "    from vinca_probe import VALUE\n"
            "    assert VALUE == 42\n"
            "    from ament_index_python.packages import get_package_prefix\n"
            "    data['workspace_prefix'] = get_package_prefix('ros_workspace')\n"
            "Path(sys.argv[1]).write_text(json.dumps(data))\n"
        )
        driver = tmp_path / "test.sh"
        driver.write_text(
            r"""set -e
conda_exe=$(cygpath -u "$1")
prefix=$(cygpath -u "$2")
probe=$(cygpath -u "$3")
out=$(cygpath -u "$4")
eval "$("$conda_exe" shell.bash hook)"
# conda-build must not run ROS hooks.
export CONDA_PREFIX="$prefix" SYS_PREFIX=building
source "$prefix/etc/conda/activate.d/ros-jazzy-ros-workspace_activate.sh"
"$prefix/python.exe" "$probe" "$out/build.json" inactive
unset SYS_PREFIX CONDA_PREFIX
conda activate "$prefix"
"$prefix/python.exe" "$probe" "$out/first.json" active
conda activate "$prefix"
"$prefix/python.exe" "$probe" "$out/second.json" active
conda deactivate
"$prefix/python.exe" "$probe" "$out/deactivated.json" inactive
""",
            newline="\n",
        )
        result = subprocess.run(
            [
                str(bash),
                "--noprofile",
                "--norc",
                str(driver),
                str(conda),
                str(prefix),
                str(probe),
                str(tmp_path),
            ],
            env=env,
            capture_output=True,
            text=True,
            timeout=300,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        first = json.loads((tmp_path / "first.json").read_text())
        second = json.loads((tmp_path / "second.json").read_text())
        assert first["ROS_DISTRO"] == "jazzy"
        assert first["ROS_VERSION"] == "2"
        assert first["ROS_PYTHON_VERSION"] == "3"
        assert first["ROS_OS_OVERRIDE"] == "conda:win64"
        for variable, expected in {
            "ROS_ETC_DIR": prefix / "Library/etc/ros",
            "AMENT_PREFIX_PATH": prefix / "Library",
            "AMENT_PYTHON_EXECUTABLE": prefix / "python.exe",
            "QT_PLUGIN_PATH": prefix / "Library/plugins",
            "workspace_prefix": prefix / "Library",
            "CMAKE_PREFIX_PATH": extra_path,
        }.items():
            assert Path(first[variable]) == expected, (variable, first[variable])
        python_paths = first["PYTHONPATH"].split(";")
        assert python_paths and all(python_paths)
        assert len(python_paths) == len(set(python_paths))
        assert any(
            Path(p) == prefix / "Library/lib/site-packages" for p in python_paths
        )
        assert Path(python_paths[0]) == extra_path
        assert sum(Path(p) == extra_path for p in first["PATH"].split(";")) == 1
        native_paths = [Path(p) for p in first["PATH"].split(";")]
        if path_operation == "append-non-duplicate":
            assert native_paths[-1] == extra_path
        else:
            assert native_paths[0] == extra_path
        if path_operation == "set":
            assert native_paths == [extra_path, bash.parent.parent / "usr/bin"]
        assert {key: first[key] for key in ROS_VARIABLES} == {
            key: second[key] for key in ROS_VARIABLES
        }
        assert first["PATH"] == second["PATH"]
        for state in ("build", "deactivated"):
            values = json.loads((tmp_path / f"{state}.json").read_text())
            assert not {key: values[key] for key in ROS_VARIABLES if key in values}
    finally:
        marker.unlink(missing_ok=True)
        (package_dir / "package.dsv").unlink(missing_ok=True)
        if package_dir.exists():
            package_dir.rmdir()
        for path, content in originals.items():
            if content is None:
                path.unlink(missing_ok=True)
            else:
                path.write_bytes(content)
