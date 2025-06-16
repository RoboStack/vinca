import datetime
import shutil
import os
import re
import stat

from ruamel import yaml
from pathlib import Path

from vinca.utils import get_pkg_build_number

TEMPLATE = """\
# yaml-language-server: $schema=https://raw.githubusercontent.com/prefix-dev/recipe-format/main/schema.json

package:
  name: ros
  version: 0.0.1

source:

build:
  number: 0

about:
  homepage: https://www.ros.org/
  license: BSD-3-Clause
  summary: |
    Robot Operating System

extra:
  recipe-maintainers:
    - ros-forge

"""

post_process_items = [
    {
        "files": ["*.pc"],
        "regex": '(?:-L|-I)?"?([^;\\s]+/sysroot/)',
        "replacement": "$$(CONDA_BUILD_SYSROOT_S)",
    },
    {
        "files": ["*.cmake"],
        "regex": '([^;\\s"]+/sysroot)',
        "replacement": "$$ENV{CONDA_BUILD_SYSROOT}",
    },
    {
        "files": ["*.cmake"],
        "regex": '([^;\\s"]+/MacOSX\\d*\\.?\\d*\\.sdk)',
        "replacement": "$$ENV{CONDA_BUILD_SYSROOT}",
    },
]


def write_recipe_package(recipe):
    file = yaml.YAML()
    file.width = 4096
    file.indent(mapping=2, sequence=4, offset=2)

    os.makedirs(recipe["package"]["name"], exist_ok=True)
    recipe_path = os.path.join(recipe["package"]["name"], "recipe.yaml")
    with open(recipe_path, "w") as stream:
        file.dump(recipe, stream)

def copyfile_with_exec_permissions(source_file, destination_file):
    shutil.copyfile(source_file, destination_file)

    # It seems that rattler-build requires script to have executable permissions
    if os.name == 'posix':
        # Retrieve current permissions
        current_permissions = os.stat(destination_file).st_mode
        # Set executable permissions for user, group, and others
        os.chmod(destination_file, current_permissions | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

def write_recipe(source, outputs, vinca_conf, single_file=True):
    # single_file = False
    if single_file:
        file = yaml.YAML()
        file.width = 4096
        file.indent(mapping=2, sequence=4, offset=2)
        meta = file.load(TEMPLATE)

        meta["source"] = [source[k] for k in source]
        meta["outputs"] = outputs
        meta["package"]["version"] = f"{datetime.datetime.now():%Y.%m.%d}"
        meta["recipe"] = meta["package"]
        del meta["package"]
        meta["build"]["number"] = vinca_conf.get("build_number", 0)
        meta["build"]["post_process"] = post_process_items
        with open("recipe.yaml", "w") as stream:
            file.dump(meta, stream)
    else:
        for o in outputs:
            file = yaml.YAML()
            file.width = 4096
            file.indent(mapping=2, sequence=4, offset=2)
            meta = file.load(TEMPLATE)

            # safe lookup of source entry; dummy recipes may not have a source
            # only include source section if entry is present, else remove it (dummy recipes)
            src_entry = source.get(o["package"]["name"], {})
            if src_entry:
                meta["source"] = src_entry
            else:
                meta.pop("source", None)
            for k, v in o.items():
                meta[k] = v

            meta["package"]["name"] = o["package"]["name"]
            meta["package"]["version"] = o["package"]["version"]

            meta["build"]["number"] = get_pkg_build_number(vinca_conf.get("build_number", 0), o["package"]["name"], vinca_conf)
            meta["build"]["post_process"] = post_process_items

            if test := vinca_conf["_tests"].get(o["package"]["name"]):
                print("Using test: ", test)
                text = test.read_text()
                test_content = yaml.safe_load(text)
                meta["tests"] = test_content["tests"]

            recipe_dir = (Path("recipes") / o["package"]["name"]).absolute()
            os.makedirs(recipe_dir, exist_ok=True)
            with open(recipe_dir / "recipe.yaml", "w") as stream:
                file.dump(meta, stream)

            if meta.get("source") and meta["source"].get("patches"):
                for p in meta["source"]["patches"]:
                    patch_dir, _ = os.path.split(p)
                    os.makedirs(recipe_dir / patch_dir, exist_ok=True)
                    shutil.copyfile(p, recipe_dir / p)

            build_scripts = re.findall(r"'(.*?)'", meta["build"]["script"])
            baffer = meta["build"]["script"]
            for script in build_scripts:
                script_filename = script.replace("$RECIPE_DIR", "").replace("%RECIPE_DIR%", "").replace("/", "").replace("\\", "")
                copyfile_with_exec_permissions(script_filename, recipe_dir / script_filename)
            if "catkin" in o["package"]["name"] or "workspace" in o["package"]["name"]:
                shutil.copyfile("activate.sh", recipe_dir / "activate.sh")
                shutil.copyfile("activate.bat", recipe_dir / "activate.bat")
                shutil.copyfile("activate.ps1", recipe_dir / "activate.ps1")
                shutil.copyfile("deactivate.sh", recipe_dir / "deactivate.sh")
                shutil.copyfile("deactivate.bat", recipe_dir / "deactivate.bat")
                shutil.copyfile("deactivate.ps1", recipe_dir / "deactivate.ps1")


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

    # It seems that rattler-build requires script to have executable permissions
    # See https://github.com/RoboStack/ros-humble/pull/229#issuecomment-2549988298
    if os.name == 'posix':
        # Retrieve current permissions
        current_permissions = os.stat(template_out.name).st_mode
        # Set executable permissions for user, group, and others
        os.chmod(template_out.name, current_permissions | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

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

    template_in = pkg_resources.resource_filename("vinca", "templates/activate.ps1.in")
    generate_template(template_in, open("activate.ps1", "w"))
    template_in = pkg_resources.resource_filename(
        "vinca", "templates/deactivate.ps1.in"
    )
    generate_template(template_in, open("deactivate.ps1", "w"))

    template_in = pkg_resources.resource_filename("vinca", "templates/activate.sh.in")
    generate_template(template_in, open("activate.sh", "w"))
    template_in = pkg_resources.resource_filename("vinca", "templates/deactivate.sh.in")
    generate_template(template_in, open("deactivate.sh", "w"))
