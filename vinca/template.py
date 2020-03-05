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
