from vinca.variants import get_recipe_variants


def test_get_recipe_variants_prunes_to_recipe_dependencies_and_expressions():
    recipe = {
        "requirements": {
            "build": ["${{ compiler('cxx') }}", "${{ stdlib('c') }}"],
            "host": ["python", "libfoo >=1"],
            "run": ["python"],
        },
        "build": {
            "script": "echo ${{ cxx_compiler_version }}",
        },
    }
    config = {
        "_variant_config": {
            "python": ["3.12.* *_cpython"],
            "python_impl": ["cpython"],
            "libfoo": ["2"],
            "unused": ["1"],
            "c_compiler": ["gcc"],
            "c_compiler_version": ["14"],
            "cxx_compiler": ["gxx"],
            "cxx_compiler_version": ["14"],
            "c_stdlib": ["sysroot"],
            "c_stdlib_version": ["2.17"],
            "zip_keys": [["python", "python_impl"], ["libfoo", "unused"]],
            "pin_run_as_build": {
                "python": {"max_pin": "x.x"},
                "unused": {"max_pin": "x"},
            },
        },
        "_pkg_additional_info": {},
    }

    assert get_recipe_variants(recipe, "demo", config) == {
        "python": ["3.12.* *_cpython"],
        "libfoo": ["2"],
        "cxx_compiler": ["gxx"],
        "cxx_compiler_version": ["14"],
        "c_stdlib": ["sysroot"],
        "c_stdlib_version": ["2.17"],
        "pin_run_as_build": {"python": {"max_pin": "x.x"}},
    }


def test_get_recipe_variants_merges_and_keeps_package_overrides():
    config = {
        "_variant_config": {
            "python": ["3.12.* *_cpython"],
            "unused": ["1"],
        },
        "_pkg_additional_info": {
            "demo": {
                "variant_overrides": {
                    "python": ["3.13.* *_cpython"],
                    "custom_feature": [True],
                }
            }
        },
    }

    assert get_recipe_variants(
        {"requirements": {"host": ["python"]}}, "demo", config
    ) == {
        "python": ["3.13.* *_cpython"],
        "custom_feature": [True],
    }


def test_get_recipe_variants_trims_zip_keys_to_used_members():
    config = {
        "_variant_config": {
            "python": ["3.12", "3.13"],
            "numpy": ["1", "2"],
            "python_impl": ["cpython", "cpython"],
            "zip_keys": [["python", "numpy", "python_impl"]],
        },
        "_pkg_additional_info": {},
    }

    assert get_recipe_variants(
        {"requirements": {"host": ["python", "numpy"]}}, "demo", config
    ) == {
        "python": ["3.12", "3.13"],
        "numpy": ["1", "2"],
        "zip_keys": [["python", "numpy"]],
    }


def test_get_recipe_variants_uses_test_requirements_not_test_commands():
    config = {
        "_variant_config": {
            "python": ["3.12"],
            "pytest": ["8"],
        },
        "_pkg_additional_info": {},
    }

    assert get_recipe_variants(
        {
            "tests": [
                {
                    "script": ["python -c 'print(1)'"],
                    "requirements": {"run": ["pytest"]},
                }
            ]
        },
        "demo",
        config,
    ) == {"pytest": ["8"]}
