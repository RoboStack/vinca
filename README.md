# vinca
 Conda recipe generator for ROS packages
 
**WARNING**:
This project is at a super early stage.
No guarantees everything works.

## Concept

The tool is helping generate one big `conda` recipe to capture all the selected packages.
And later one can use `conda-smithy` to auto-generate pipeline to release `conda` binary packages.

## Example

First you have to create a `vinca.yaml` to describe ROS information for `vinca` to do the magic generating the recipe.

```yaml
# vinca.yaml
ros_distro: eloquent
ros_python_version: 3

# both .rosinstall and .repos are supported
repos: 2020.03.04.rosinstall

# mapping of rosdep keys
# it can be also used to shadow the unwanted packages.
conda_index:
  - conda-forge.yaml
  - packages-ignore.yaml

# packages to skip
packages_skip_by_deps:

# packages to include
packages_select_by_deps:
  - ament_cmake

# patch directory
patch_dir: ./patch
```

## TODO
  - Implement patch_dir.
  - Add bld_cmake.bat and bld_python.bat template.
