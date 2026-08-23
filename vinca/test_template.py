from vinca.template import get_recipe_variants


def test_get_recipe_variants_merges_package_overrides():
    config = {
        "ros_distro": "jazzy",
        "_variant_config": {
            "python": ["3.12.* *_cpython"],
            "c_compiler": ["gcc", "clang"],
        },
        "_pkg_additional_info": {
            "demo_nodes_cpp": {
                "variant_overrides": {
                    "python": ["3.13.* *_cpython"],
                    "numpy": ["2"],
                }
            }
        },
    }

    variants = get_recipe_variants("ros-jazzy-demo-nodes-cpp", config)

    assert variants == {
        "python": ["3.13.* *_cpython"],
        "c_compiler": ["gcc", "clang"],
        "numpy": ["2"],
    }
    assert config["_variant_config"]["python"] == ["3.12.* *_cpython"]


def test_get_recipe_variants_uses_repository_config_without_overrides():
    config = {
        "ros_distro": "jazzy",
        "_variant_config": {"python": ["3.12.* *_cpython"]},
        "_pkg_additional_info": {},
    }

    assert get_recipe_variants("ros-jazzy-demo-nodes-cpp", config) == {
        "python": ["3.12.* *_cpython"]
    }
