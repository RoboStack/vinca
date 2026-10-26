The Windows CI job installs `environment.yml` into a disposable conda environment
and runs `vinca/test_windows_activation.py` using Git for Windows Bash. The test
renders and installs the PR's hooks, then activates the environment through real
conda. It checks the conda-build guard, ROS variables, native Python's ament index
lookup, repeated activation, and deactivation. It restores the installed hooks
afterward. An additional package with real `.dsv` hooks exercises PATH prepends,
appends and assignments, PYTHONPATH prepends, a CMAKE_PREFIX_PATH append, and paths
containing spaces. The assignment case replaces the inherited PATH with an
explicit Windows path list and checks its exact contents and order.

To reproduce in PowerShell from the repository root:

```powershell
pixi exec --spec conda -- conda env create --yes --prefix "$PWD/.pixi/ros-activation" --file tests/windows/environment.yml
$env:VINCA_WINDOWS_ROS_PREFIX = "$PWD/.pixi/ros-activation"
pixi run pytest -v vinca/test_windows_activation.py
```

The integration test skips unless `VINCA_WINDOWS_ROS_PREFIX` is set, and skips on
non-Windows platforms. Once a prefix is supplied, missing dependencies or Git Bash
fail the test rather than skipping it. Use a disposable environment: the test
temporarily replaces its ros_workspace activation hooks.

This test does not run a full rclcpp CMake build.
