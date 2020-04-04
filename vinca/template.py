import datetime
import ruamel
import os

TEMPLATE = """\
package:
  name: ros
  version: 0.0.1

source:

build:
  number: 0
  skip: true  # [not win64]

outputs:

about:
  home: https://www.ros.org/
  license: BSD-3-Clause
  summary: |
    Robot Operating System

extra:
  recipe-maintainers:
    - ros-forge
"""


def write_recipe(source, outputs):
    yaml = ruamel.yaml.YAML()
    yaml.indent(mapping=2, sequence=4, offset=2)
    meta = yaml.load(TEMPLATE)

    meta['source'] = source
    meta['outputs'] = outputs
    meta['package']['version'] = f"{datetime.datetime.now():%Y.%m.%d}"

    with open("meta.yaml", 'w') as stream:
        yaml.dump(meta, stream)


def generate_template(template_in, template_out):
    import em
    g = {
      'ros_distro': os.environ['ROS_DISTRO']
    }
    interpreter = em.Interpreter(
      output=template_out,
      options={em.RAW_OPT: True, em.BUFFERED_OPT: True})
    interpreter.updateGlobals(g)
    interpreter.file(open(template_in))
    interpreter.shutdown()


def generate_bld_ament_cmake():
    import pkg_resources
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/bld_ament_cmake.bat.in')
    generate_template(template_in, open('bld_ament_cmake.bat', 'w'))


def generate_bld_ament_python():
    import pkg_resources
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/bld_ament_python.bat.in')
    generate_template(template_in, open('bld_ament_python.bat', 'w'))


def generate_bld_catkin():
    import pkg_resources
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/bld_catkin.bat.in')
    generate_template(template_in, open('bld_catkin.bat', 'w'))


def generate_bld_colcon_merge():
    import pkg_resources
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/bld_colcon_merge.bat.in')
    generate_template(template_in, open('bld_colcon_merge.bat', 'w'))


def generate_activate_hook():
    import pkg_resources
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/activate.bat.in')
    generate_template(template_in, open('activate.bat', 'w'))
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/deactivate.bat.in')
    generate_template(template_in, open('deactivate.bat', 'w'))
