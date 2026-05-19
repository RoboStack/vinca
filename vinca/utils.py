import yaml
import hashlib
import os
import time
import json
import requests


class folded_unicode(str):
    pass


class literal_unicode(str):
    pass


def folded_unicode_representer(dumper, data):
    return dumper.represent_scalar("tag:yaml.org,2002:str", data, style=">")


def literal_unicode_representer(dumper, data):
    return dumper.represent_scalar("tag:yaml.org,2002:str", data, style="|")


yaml.add_representer(folded_unicode, folded_unicode_representer)
yaml.add_representer(literal_unicode, literal_unicode_representer)


class NoAliasDumper(yaml.SafeDumper):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.add_representer(folded_unicode, folded_unicode_representer)
        self.add_representer(literal_unicode, literal_unicode_representer)

    def ignore_aliases(self, data):
        return True


def get_repodata(url_or_path, platform=None):
    if platform:
        if not url_or_path.endswith("/"):
            url_or_path += "/"
        url_or_path += f"{platform}/repodata.json"

    if "://" not in url_or_path:
        if not os.path.exists(url_or_path):
            print(f"No repodata found at {url_or_path}, assuming no existing packages")
            return {"packages": {}, "packages.conda": {}}
        with open(url_or_path) as fi:
            return json.load(fi)
    print("Downloading repodata from ", url_or_path)

    m = hashlib.md5(url_or_path.encode("utf-8")).hexdigest()[:10]
    # print(tempfile.gettempdir())
    fn = f"vinca_{m}.json"
    if os.path.exists(fn):
        st = os.stat(fn)
        age = time.time() - st.st_mtime
        print(f"Found cached repodata, age: {age}")
        max_age = 100_000  # seconds == 27 hours
        if age < max_age:
            with open(fn) as fi:
                try:
                    return json.load(fi)
                except json.JSONDecodeError:
                    print(f"Ignoring invalid cached repodata at {fn}")
                    os.remove(fn)

    repodata = requests.get(url_or_path)
    if repodata.status_code == 404:
        print(f"No repodata found at {url_or_path}, assuming no existing packages")
        return {"packages": {}, "packages.conda": {}}
    repodata.raise_for_status()
    content = repodata.content
    if not content.strip():
        print(f"No repodata found at {url_or_path}, assuming no existing packages")
        return {"packages": {}, "packages.conda": {}}
    try:
        parsed_repodata = json.loads(content)
    except json.JSONDecodeError:
        print(
            f"No valid repodata found at {url_or_path}, assuming no existing packages"
        )
        return {"packages": {}, "packages.conda": {}}

    with open(fn, "w") as fcache:
        fcache.write(content.decode("utf-8"))
    return parsed_repodata


def ensure_name_is_without_distro_prefix_and_with_underscores(name, vinca_conf):
    """
    Ensure that the name is without distro prefix and with underscores
    e.g. "ros-humble-pkg-name" -> "pkg_name"
    """
    newname = name.replace("-", "_")
    distro_prefix = "ros_" + vinca_conf.get("ros_distro") + "_"
    if newname.startswith(distro_prefix):
        newname = newname.replace(distro_prefix, "")

    return newname


def get_pkg_additional_info(pkg_name, vinca_conf):
    normalized_name = ensure_name_is_without_distro_prefix_and_with_underscores(
        pkg_name, vinca_conf
    )
    pkg_additional_info_all = vinca_conf["_pkg_additional_info"]
    if pkg_additional_info_all is None:
        pkg_additional_info = {}
    else:
        pkg_additional_info = vinca_conf.get("_pkg_additional_info", {}).get(
            normalized_name, {}
        )
    return pkg_additional_info


def get_pkg_build_number(default_build_number, pkg_name, vinca_conf):
    pkg_additional_info = get_pkg_additional_info(pkg_name, vinca_conf)
    return pkg_additional_info.get("build_number", default_build_number)


# Return true if the package is actually provided in conda-forge, and so we generate
# only a recipe with a run dependency on the conda forge package
def is_dummy_metapackage(pkg_name, vinca_conf):
    pkg_additional_info = get_pkg_additional_info(pkg_name, vinca_conf)
    if pkg_additional_info.get("generate_dummy_package_with_run_deps"):
        return True
    else:
        return False
