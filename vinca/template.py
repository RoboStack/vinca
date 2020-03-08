import datetime
import ruamel

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
    - seanyen
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
    g = {}
    interpreter = em.Interpreter(
      output=template_out,
      options={em.RAW_OPT: True, em.BUFFERED_OPT: True})
    interpreter.updateGlobals(g)
    interpreter.file(open(template_in))
    interpreter.shutdown()


def generate_bld_cmake():
    import pkg_resources
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/bld_cmake.bat.in')
    generate_template(template_in, open('bld_cmake.bat', 'w'))


def generate_bld_python():
    import pkg_resources
    template_in = pkg_resources.resource_filename(
      'vinca', 'templates/bld_python.bat.in')
    generate_template(template_in, open('bld_python.bat', 'w'))
