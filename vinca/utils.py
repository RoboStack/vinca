import yaml
import hashlib
import os
import time
import json
import requests
import ruamel.yaml


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
        with open(url_or_path) as fi:
            return json.load(fi)

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
                return json.load(fi)

    repodata = requests.get(url_or_path)
    content = repodata.content
    with open(fn, "w") as fcache:
        fcache.write(content.decode("utf-8"))
    return json.loads(content)


def get_pinnings(url_or_path):
    yaml = ruamel.yaml.YAML()
    yaml.allow_duplicate_keys = True

    if "://" not in url_or_path:
        with open(url_or_path) as fi:
            return yaml.load(fi)

    response = requests.get(url_or_path)
    return yaml.load(response.content)
