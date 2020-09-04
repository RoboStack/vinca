import datetime
import ruamel
import os

from ruamel import yaml

TEMPLATE = """\
package:
  name: ros
  version: 0.0.1

source:

build:
  number: 0

about:
  home: https://www.ros.org/
  license: BSD-3-Clause
  summary: |
    Robot Operating System

extra:
  recipe-maintainers:
    - ros-forge

"""

def write_recipe_package(recipe):
    file = yaml.YAML()
    file.indent(mapping=2, sequence=4, offset=2)
    with open(recipe['package']['name'] + '.yaml', 'w') as stream:
        file.dump(recipe, stream)

def write_recipe(source, outputs, build_number=0, single_file=True):
    # single_file = False
    if single_file:
        file = yaml.YAML()
        file.indent(mapping=2, sequence=4, offset=2)
        meta = file.load(TEMPLATE)

        meta['source'] = [source[k] for k in source]
        meta['outputs'] = outputs
        meta['package']['version'] = f"{datetime.datetime.now():%Y.%m.%d}"
        meta['build']['number'] = build_number
        with open("recipe.yaml", 'w') as stream:
            file.dump(meta, stream)
    else:
        for o in outputs:
            file = yaml.YAML()
            file.indent(mapping=2, sequence=4, offset=2)
            meta = file.load(TEMPLATE)

            meta['source'] = source[o['package']['name']]
            for k, v in o.items():
                meta[k] = v

            meta['package']['name'] = o['package']['name']
            meta['package']['version'] = o['package']['version']

            meta['build']['number'] = build_number

            if not os.path.isdir("recipes"):
                os.makedirs("recipes")
            with open(os.path.join("recipes", f"{o['package']['name']}.yaml"), 'w') as stream:
                file.dump(meta, stream)

def generate_template(template_in, template_out):
    import em
    g = {
      'ros_distro': 'melodic' if not os.environ.get('ROS_DISTRO', None) else os.environ['ROS_DISTRO']
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
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/build_catkin.sh.in')
    generate_template(template_in, open('build_catkin.sh', 'w'))


def generate_bld_colcon_merge():
    import pkg_resources
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/bld_colcon_merge.bat.in')
    generate_template(template_in, open('bld_colcon_merge.bat', 'w'))


def generate_bld_catkin_merge():
    import pkg_resources
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/bld_catkin_merge.bat.in')
    generate_template(template_in, open('bld_catkin_merge.bat', 'w'))


def generate_activate_hook():
    import pkg_resources
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/activate.bat.in')
    generate_template(template_in, open('activate.bat', 'w'))
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/deactivate.bat.in')
    generate_template(template_in, open('deactivate.bat', 'w'))
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/activate.sh.in')
    generate_template(template_in, open('activate.sh', 'w'))
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/deactivate.sh.in')
    generate_template(template_in, open('deactivate.sh', 'w'))
