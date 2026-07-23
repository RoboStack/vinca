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
