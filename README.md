# vinca

rattler-build recipe (i.e. conda recipe v1) generator for ROS packages

**WARNING**:
This project is actively mantained and can frequently change based on the needs of the RoboStack project.

## Concept

The tool generates `conda` rattler-build recipes to capture all the selected ROS packages.

## Example

The repo contains a `vinca` tool that reads a `vinca.yaml` file that contains all its metadata.

For an up-to-date example of how to write a `vinca.yaml`, check the repos of the mantained RoboStack distros:
* https://github.com/RoboStack/ros-noetic/
* https://github.com/RoboStack/ros-humble
* https://github.com/RoboStack/ros-jazzy/
