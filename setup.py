from setuptools import find_packages, setup

args = {
    'include_package_data': True,
    'packages': find_packages(),
    'package_data': {
        'vinca': [
            'templates/*'
        ]
    },
}

setup(**args)
