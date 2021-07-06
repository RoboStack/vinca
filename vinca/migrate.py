import json
import requests
import networkx as nx

packages_to_migrate = ["libopencv"]
distro = "noetic"

arches = ["linux-64", "linux-aarch64", "win-64", "osx-64", "osx-arm64"]
arches = ["linux-64"]

for arch in arches:
    url = f"https://conda.anaconda.org/robostack/{arch}/repodata.json"
    print("URL: ", url)
    # return
    repodata = requests.get(url).json()
    packages = repodata["packages"]
    to_migrate = set()
    ros_pkgs = set()
    ros_prefix = f"ros-{distro}"
    for pkey in packages:
        if not pkey.startswith(ros_prefix):
            continue

        pname = pkey.rsplit('-', 2)[0]
        ros_pkgs.add(pname)

        p = packages[pkey]

        for d in p.get("depends", []):
            if d.split()[0] in packages_to_migrate:
                # print(f"need to migrate {pkey}")
                to_migrate.add(pname)

    # print(to_migrate)
    # print(ros_pkgs)

    latest = {}
    for pkg in ros_pkgs:
        current = current_version = None
        for pkey in packages:
            if packages[pkey]["name"] == pkg:
                tmp = packages[pkey]["version"].split('.')
                version = []
                for el in tmp:
                    if el.isdecimal():
                        version.append(int(el))
                    else:
                        x = re.search(r'[^0-9]', version).start()
                        version.append(int(el[:x]))

                version = tuple(version)

                if not current or version > current_version:
                    current_version = version
                    current = pkey
        latest[pkg] = current

    # print(latest)

        # now we can build the graph ... 

    G = nx.DiGraph()
    for pkg, pkgkey in latest.items():
        full_pkg = packages[pkgkey]
        for dep in full_pkg.get("depends", []):
            req = dep.split(' ')[0]
            G.add_node(pkg)
            if req.startswith(ros_prefix):
                G.add_edge(pkg, req)

    gsorted = nx.topological_sort(G)
    gsorted = list(reversed([g for g in gsorted]))

    to_migrate = sorted(to_migrate, key=lambda x: gsorted.index(x))

    print("Sorted to migrate: ", to_migrate)
    
    # import matplotlib.pyplot as plt
    # nx.draw(G, with_labels=True, font_weight='bold')
    # plt.show()


