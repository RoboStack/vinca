import datetime
import shutil
import ruamel
import os

from ruamel import yaml
from pathlib import Path

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
    file.width = 4096
    file.indent(mapping=2, sequence=4, offset=2)

    os.makedirs(recipe["package"]["name"], exist_ok=True)
    recipe_path = os.path.join(recipe["package"]["name"], "recipe.yaml")
    with open(recipe_path, "w") as stream:
        file.dump(recipe, stream)


def write_recipe(source, outputs, build_number=0, single_file=True):
    # single_file = False
    if single_file:
        file = yaml.YAML()
        file.width = 4096
        file.indent(mapping=2, sequence=4, offset=2)
        meta = file.load(TEMPLATE)

        meta["source"] = [source[k] for k in source]
        meta["outputs"] = outputs
        meta["package"]["version"] = f"{datetime.datetime.now():%Y.%m.%d}"
        meta["build"]["number"] = build_number
        with open("recipe.yaml", "w") as stream:
            file.dump(meta, stream)
    else:
        for o in outputs:
            file = yaml.YAML()
            file.width = 4096
            file.indent(mapping=2, sequence=4, offset=2)
            meta = file.load(TEMPLATE)

            meta["source"] = source[o["package"]["name"]]
            for k, v in o.items():
                meta[k] = v

            meta["package"]["name"] = o["package"]["name"]
            meta["package"]["version"] = o["package"]["version"]

            meta["build"]["number"] = build_number

            recipe_dir = (Path("recipes") / o["package"]["name"]).absolute()
            os.makedirs(recipe_dir, exist_ok=True)
            with open(recipe_dir / "recipe.yaml", "w") as stream:
                file.dump(meta, stream)

            if meta["source"].get("patches"):
                for p in meta["source"]["patches"]:
                    patch_dir, _ = os.path.split(p)
                    os.makedirs(recipe_dir / patch_dir, exist_ok=True)
                    shutil.copyfile(p, recipe_dir / p)

            for key, script in meta["build"]["script"].items():
                shutil.copyfile(script, recipe_dir / script)
            if "catkin" in o["package"]["name"] or "workspace" in o["package"]["name"]:
                shutil.copyfile("activate.sh", recipe_dir / "activate.sh")
                shutil.copyfile("activate.bat", recipe_dir / "activate.bat")
                shutil.copyfile("deactivate.sh", recipe_dir / "deactivate.sh")
                shutil.copyfile("deactivate.bat", recipe_dir / "deactivate.bat")


def generate_template(template_in, template_out):
    import em
    from vinca.config import skip_testing, ros_distro

    g = {"ros_distro": ros_distro, "skip_testing": "ON" if skip_testing else "OFF"}
    interpreter = em.Interpreter(
        output=template_out, options={em.RAW_OPT: True, em.BUFFERED_OPT: True}
    )
    interpreter.updateGlobals(g)
    interpreter.file(open(template_in))
    interpreter.shutdown()


def generate_bld_ament_cmake():
    import pkg_resources

    template_in = pkg_resources.resource_filename(
        "vinca", "templates/bld_ament_cmake.bat.in"
    )
    generate_template(template_in, open("bld_ament_cmake.bat", "w"))
    template_in = pkg_resources.resource_filename(
        "vinca", "templates/build_ament_cmake.sh.in"
    )
    generate_template(template_in, open("build_ament_cmake.sh", "w"))


def generate_bld_ament_python():
    import pkg_resources

    template_in = pkg_resources.resource_filename(
        "vinca", "templates/bld_ament_python.bat.in"
    )
    generate_template(template_in, open("bld_ament_python.bat", "w"))
    template_in = pkg_resources.resource_filename(
        "vinca", "templates/build_ament_python.sh.in"
    )
    generate_template(template_in, open("build_ament_python.sh", "w"))


def generate_bld_catkin():
    import pkg_resources

    template_in = pkg_resources.resource_filename(
        "vinca", "templates/bld_catkin.bat.in"
    )
    generate_template(template_in, open("bld_catkin.bat", "w"))
    template_in = pkg_resources.resource_filename(
        "vinca", "templates/build_catkin.sh.in"
    )
    generate_template(template_in, open("build_catkin.sh", "w"))


def generate_bld_colcon_merge():
    import pkg_resources

    template_in = pkg_resources.resource_filename(
        "vinca", "templates/bld_colcon_merge.bat.in"
    )
    generate_template(template_in, open("bld_colcon_merge.bat", "w"))


def generate_bld_catkin_merge():
    import pkg_resources

    template_in = pkg_resources.resource_filename(
        "vinca", "templates/bld_catkin_merge.bat.in"
    )
    generate_template(template_in, open("bld_catkin_merge.bat", "w"))


def generate_activate_hook():
    import pkg_resources

    template_in = pkg_resources.resource_filename("vinca", "templates/activate.bat.in")
    generate_template(template_in, open("activate.bat", "w"))
    template_in = pkg_resources.resource_filename(
        "vinca", "templates/deactivate.bat.in"
    )
    generate_template(template_in, open("deactivate.bat", "w"))
    template_in = pkg_resources.resource_filename("vinca", "templates/activate.sh.in")
    generate_template(template_in, open("activate.sh", "w"))
    template_in = pkg_resources.resource_filename("vinca", "templates/deactivate.sh.in")
    generate_template(template_in, open("deactivate.sh", "w"))
