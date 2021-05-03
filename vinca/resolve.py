import os
from urllib.request import urlopen
import platform
from vinca import config

map_platform_python_to_conda = {
    "linux-64": "linux",
    "linux-aarch64": "linux",
    "osx-64": "osx",
    "win-64": "win64",
}


def get_conda_index(vinca_conf, base_dir):
    import ruamel.yaml

    yaml = ruamel.yaml.YAML()
    conda_index = []
    for i in vinca_conf["conda_index"]:
        ip = os.path.join(base_dir, i)
        if os.path.isfile(ip):
            rawdata = yaml.load(open(ip, "r"))
        else:
            rawdata = yaml.load(urlopen(i))
        conda_index.append(rawdata)
    return conda_index


def resolve_pkgname_from_indexes(pkg_shortname, conda_index):
    for i in conda_index:
        if pkg_shortname in i:
            sys_platform = map_platform_python_to_conda[config.selected_platform]
            if "robostack" in i[pkg_shortname].keys():
                if sys_platform in i[pkg_shortname]["robostack"]:
                    return i[pkg_shortname]["robostack"][sys_platform]
                else:
                    return i[pkg_shortname]["robostack"]
            raise KeyError(
                "Missing package for platform {}: {}\nCheck your conda metadata!".format(
                    sys_platform, pkg_shortname
                )
            )

    return None


def resolve_pkgname(pkg_shortname, vinca_conf, distro, is_rundep=False):
    pkg_names = resolve_pkgname_from_indexes(
        pkg_shortname, vinca_conf["_conda_indexes"]
    )
    if pkg_names is None:
        if not distro.check_package(pkg_shortname):
            return []
        else:
            if (
                "packages_remove_from_deps" in vinca_conf
                and vinca_conf["packages_remove_from_deps"] is not None
                and pkg_shortname.replace("_", "-")
                not in vinca_conf["packages_remove_from_deps"]
            ):
                return [
                    "ros-%s-%s"
                    % (vinca_conf["ros_distro"], pkg_shortname.replace("_", "-"))
                ]
            else:
                return []
    else:
        if is_rundep:  # for run dependencies, remove the version
            pkg_names_pinned = []

            for pkg_name in pkg_names:
                if " " in pkg_name:
                    pkg_name_raw = pkg_name.split(" ")[0]
                else:
                    pkg_name_raw = pkg_name

                if pkg_name_raw != "python":
                    pkg_names_pinned.append(pkg_name_raw)

            return pkg_names_pinned
        else:  # for host deps
            return pkg_names
