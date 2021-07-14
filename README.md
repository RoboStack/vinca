# vinca
 Conda recipe generator for ROS packages
 
**WARNING**:
This project is at a super early stage.
No guarantees everything works.

## Concept

The tool generates `conda` recipe to capture all the selected ROS packages.
Later, one can use `conda-smithy` to auto-generate pipeline to release `conda` binary packages.

## Example

First you have to create a `vinca.yaml` to describe ROS information for `vinca` to do the magic generating the recipe.

```yaml
# vinca.yaml
ros_distro: eloquent

# optionally, a fat archive can be generated too.
fat_archive: true
name: ros-eloquent-fat-ament-cmake

# mapping of rosdep keys
# it can be also used to shadow the unwanted packages.
conda_index:
  - 'https://github.com/RoboStack/ros-galactic/blob/master/vinca_linux_64.yaml'

# packages to skip
packages_skip_by_deps:

# packages to include
packages_select_by_deps:
  - ament_cmake

# patch directory
patch_dir: ./patch
```

### Special modes

#### Build all

To generate recipes for all available ROS packages you can add `build_all: true` to your vinca.yaml. In that case, otherwise selected packages will be ignored and vinca will generate recipes for all available packages. To try to see how far one can go, it's recommended to run vinca and boa like this:

```
vinca --multiple  # to generate a `./recipes` folder with multiple subfolders
boa build --skip-existing=fast --continue-on-failure -m conda_build_config.yaml ./recipes
```
