

def get_repos_in_vcstool_format(repositories):
    repos = {}
    if repositories is None:
        return repos
    for path in repositories:
        repo = {}
        attributes = repositories[path]
        try:
            repo['type'] = attributes['type']
            repo['url'] = attributes['url']
            if 'version' in attributes:
                repo['version'] = attributes['version']
        except AttributeError:
            continue
        repos[path] = repo
    return repos


def get_repos_in_rosinstall_format(root):
    repos = {}
    for i, item in enumerate(root):
        if len(item.keys()) != 1:
            raise RuntimeError('Input data is not valid format')
        repo = {'type': list(item.keys())[0]}
        attributes = list(item.values())[0]
        try:
            path = attributes['local-name']
        except AttributeError:
            continue
        try:
            repo['url'] = attributes['uri']
            if 'version' in attributes:
                repo['version'] = attributes['version']
        except AttributeError:
            continue
        repos[path] = repo
    return repos


def get_repos(filepath):
    import ruamel.yaml
    yaml = ruamel.yaml.YAML()
    doc_root = yaml.load(open(filepath, 'r'))

    if 'repositories' in doc_root:
        # most likely it is a repos file.
        return get_repos_in_vcstool_format(doc_root['repositories'])
    else:
        return get_repos_in_rosinstall_format(doc_root)
