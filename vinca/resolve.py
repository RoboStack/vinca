def get_conda_index(vinca_conf):
    import ruamel.yaml

    yaml = ruamel.yaml.YAML()
    conda_index = []
    for i in vinca_conf['conda_index']:
        conda_index.append(yaml.load(open(i, 'r')))
    return conda_index


def resolve_pkgname_from_indexes(pkg_shortname, conda_index):
    for i in conda_index:
        if pkg_shortname in i:
            # TODO: replace with platform variable.
            return i[pkg_shortname]['win64']
    return None


def resolve_pkgname(pkg_shortname, vinca_conf):
    pkg_names = resolve_pkgname_from_indexes(
        pkg_shortname, vinca_conf['_conda_indexes'])
    if pkg_names is None:
        return ['ros-%s-%s' %
                (vinca_conf['ros_distro'],
                 pkg_shortname.replace('_', '-'))]
    else:
        return pkg_names
