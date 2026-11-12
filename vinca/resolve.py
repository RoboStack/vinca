from __future__ import annotations

import os
from typing import Optional
from urllib.request import urlopen

from vinca import config
from vinca.distro import Distro
from vinca.naming import get_package_name
from vinca.schemas import CondaIndex, VincaConfig, validate

map_platform_python_to_conda = {
    "linux-64": "linux",
    "linux-aarch64": "linux",
    "osx-64": "osx",
    "osx-arm64": "osx",
    "win-64": "win64",
    "emscripten-wasm32": "emscripten",
}


def get_conda_index(indexes: list[str], base_dir: str) -> list[CondaIndex]:
    import ruamel.yaml

    yaml = ruamel.yaml.YAML()
    conda_index = []
    for i in indexes:
        ip = os.path.join(base_dir, i)
        if os.path.isfile(ip):
            with open(ip, encoding="utf-8") as stream:
                rawdata = yaml.load(stream)
        else:
            rawdata = yaml.load(urlopen(i))
        conda_index.append(validate(CondaIndex, rawdata or {}, i))
    return conda_index


def resolve_pkgname_from_indexes(
    pkg_shortname: str, conda_index: list[CondaIndex]
) -> Optional[list[str]]:
    for i in conda_index:
        if pkg_shortname in i:
            selected_platform = config.selected_platform
            if selected_platform is None:
                raise RuntimeError(
                    "A target platform is required to resolve package names"
                )
            sys_platform = map_platform_python_to_conda[selected_platform]
            packages = i[pkg_shortname].robostack
            if isinstance(packages, list):
                return packages
            if packages is not None:
                if selected_platform in packages:
                    return packages[selected_platform]
                if sys_platform in packages:
                    return packages[sys_platform]
                return list(packages)
            raise KeyError(
                "Missing package for platform {}: {}\nCheck your conda metadata!".format(
                    sys_platform, pkg_shortname
                )
            )

    return None


def should_skip_pkg(pkg_shortname: str, vinca_conf: VincaConfig) -> bool:
    skip = vinca_conf.remove_from_deps
    return pkg_shortname in skip or pkg_shortname.replace("_", "-") in skip


def resolve_pkgname(
    pkg_shortname: str,
    vinca_conf: VincaConfig,
    distro: Distro,
    is_rundep: bool = False,
) -> list[str]:
    pkg_names = resolve_pkgname_from_indexes(pkg_shortname, vinca_conf.conda_indexes)
    if pkg_names is None:
        if not distro.check_package(pkg_shortname) or should_skip_pkg(
            pkg_shortname, vinca_conf
        ):
            return []
        else:
            return [get_package_name(pkg_shortname, distro, vinca_conf)]
    else:
        if is_rundep:  # for run dependencies, remove the version
            pkg_names_pinned = []

            for pkg_name in pkg_names:
                if " " in pkg_name and "${{" not in pkg_name:
                    pkg_name_raw = pkg_name.split(" ")[0]
                else:
                    pkg_name_raw = pkg_name

                if pkg_name_raw != "python":
                    pkg_names_pinned.append(pkg_name_raw)

            return pkg_names_pinned
        else:  # for host deps
            return pkg_names
