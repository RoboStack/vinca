import os
from urllib.request import urlopen
import platform


map_platform_python_to_conda = {
    'Linux': 'linux',
    'Darwin': 'osx',
    'Windows': 'win64'
}


def get_conda_index(vinca_conf):
    import ruamel.yaml

    yaml = ruamel.yaml.YAML()
    conda_index = []
    for i in vinca_conf['conda_index']:
        if os.path.isfile(i):
            rawdata = yaml.load(open(i, 'r'))
        else:
            rawdata = yaml.load(urlopen(i))
        conda_index.append(rawdata)
    return conda_index


def resolve_pkgname_from_indexes(pkg_shortname, conda_index):
    for i in conda_index:
        if pkg_shortname in i:
            sys_platform = map_platform_python_to_conda[platform.system()]
            if 'conda-forge' in i[pkg_shortname].keys():
                if sys_platform in i[pkg_shortname]['conda-forge']:
                    return i[pkg_shortname]['conda-forge'][sys_platform]
                elif 'unix' in i[pkg_shortname]['conda-forge'] and sys_platform in ['linux', 'osx']:
                    return i[pkg_shortname]['conda-forge']['unix']
                else:
                    return i[pkg_shortname]['conda-forge']
            raise KeyError("Missing package for platform {}: {}\nCheck your conda metadata!".format(sys_platform, pkg_shortname))

    return None


def resolve_pkgname(pkg_shortname, vinca_conf, distro, is_rundep=False):
    pkg_names = resolve_pkgname_from_indexes(
        pkg_shortname, vinca_conf['_conda_indexes'])
    if pkg_names is None:
        if not distro.check_package(pkg_shortname):
            return []
        else:
            return ['ros-%s-%s' %
                    (vinca_conf['ros_distro'],
                     pkg_shortname.replace('_', '-'))]
    else:
        if is_rundep:  # for run dependencies, remove the version
            pkg_names_pinned = []

            for pkg_name in pkg_names:
                if ' ' in pkg_name:
                    pkg_name_raw = pkg_name.split(' ')[0]
                    pkg_names_pinned.append(pkg_name_raw)
                else:
                    pkg_names_pinned.append(pkg_name)
            return pkg_names_pinned
        else:
            return pkg_names
