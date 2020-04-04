import os
from urllib.request import urlopen


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
            # TODO: replace with platform variable.
            return i[pkg_shortname]['conda-forge']
    return None


def resolve_pkgname(pkg_shortname, vinca_conf, distro):
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
        return pkg_names
