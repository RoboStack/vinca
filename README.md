# vinca

rattler-build recipe (i.e. conda recipe v1) generator for ROS packages

**WARNING**:
This project is actively mantained and can frequently change based on the needs of the RoboStack project.

## Concept

The tool generates `conda` rattler-build recipes to capture all the selected ROS packages.

## Example

The repo contains a `vinca` tool that reads a `vinca.yaml` file that contains all its metadata.

For an up-to-date example of how to write a `vinca.yaml`, check the repos of the maintained RoboStack distros:
* https://github.com/RoboStack/ros-noetic/
* https://github.com/RoboStack/ros-humble
* https://github.com/RoboStack/ros-jazzy/

## Package naming

The optional `package_name_mode` setting controls the transition from legacy distro-qualified names such as `ros-humble-rclcpp` to ROS-major-version names such as `ros2-rclcpp`:

* `legacy` (default): generate only distro-qualified package names.
* `both`: generate the new names and compatibility packages under the legacy names. Each compatibility package depends on the corresponding new package at the same version.
* `new`: generate only the new names.

Existing configurations remain on `legacy` when this setting is omitted. To start migrating a distribution to the new names, use:

```yaml
package_name_mode: both
```

Once users and downstream projects have migrated, switch to `new` to stop generating the compatibility packages. New ROS 1 package names use the `ros-` prefix; new ROS 2 package names use `ros2-`.

## Configuring setup-pixi in generated workflows

Generated GitHub Actions workflows use `prefix-dev/setup-pixi@v0` by default. Override the action version in `vinca.yaml` when needed:

```yaml
setup_pixi_version: latest
pixi_version: latest
```

`setup_pixi_version` selects the `prefix-dev/setup-pixi` action release and defaults to `v0`. `pixi_version` selects the Pixi binary installed by that action and defaults to `latest`. Semantic versions may be written with or without the `v` prefix; Vinca adds exactly one `v` when needed. Symbolic references such as `latest` and GitHub Action commit hashes are passed through unchanged, so a pinned action is also supported:

```yaml
setup_pixi_version: 0123456789abcdef0123456789abcdef01234567
pixi_version: v0.78.0
```

Pinning a release or full action commit hash is recommended for reproducible workflows.

## Managing conda-forge pinning

RoboStack repositories can keep their global pins reproducible without copying and
manually editing conda-forge's full pinning file. Add a `vinca_pinning.yaml` file:

```yaml
conda_forge_pinning_version: 2026.08.20.20.38.26
migrations:
  - libboost190
pinning_overrides:
  python:
    - 3.14.* *_cp314
  # Override every member of a zip_keys group together.
  is_python_min:
    - false
  python_impl:
    - cpython
```

Render the build-tool input with:

```shell
vinca-pinning-render
```

The renderer downloads that exact `conda-forge-pinning` package and starts with its
`conda_build_config.yaml`. It sorts the named migrations by `migrator_ts`, just as
`conda-smithy` does, and combines each one using the [CFEP-9 variant algebra](https://github.com/conda-forge/cfep/blob/main/cfep-09.md).

For a full rebuild, update the base and eligible migrations with:

```shell
vinca-pinning-update --render
```

The updater updates the base `conda_forge_pinning_version` to the latest one, but keeps `pinning_overrides` unchanged. It uses existing generated recipes
when available; otherwise it collects the would-be recipe dependencies for Linux,
macOS, and Windows without fully generating the recipes. The ROS distro model and
parsed group dependencies are reused across platforms. For each active, non-paused
conda-forge migration, it checks the public
conda-forge migration status and applies it only after all feedstocks producing the
external dependencies of the selected ROS recipes are marked done. Previously
applied migrations remain selected while they are active; migrations incorporated
into a newer base disappear from the list automatically.

Use repeatable `--platform` options to limit dependency discovery, and use `-d` when
the RoboStack repository is not the current directory.
