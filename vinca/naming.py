"""Helpers for selecting and generating ROS package names."""

from enum import Enum


class PackageNameMode(str, Enum):
    """Supported ROS package naming transition modes."""

    LEGACY = "legacy"
    BOTH = "both"
    NEW = "new"


def get_package_name_mode(vinca_conf):
    """Return the configured package name mode, defaulting to legacy names."""
    value = vinca_conf.get("package_name_mode", PackageNameMode.LEGACY.value)
    try:
        return PackageNameMode(value)
    except ValueError as error:
        choices = ", ".join(mode.value for mode in PackageNameMode)
        raise ValueError(
            f"Invalid package_name_mode {value!r}; expected one of: {choices}"
        ) from error


def get_package_prefix(distro, vinca_conf):
    """Return the prefix used for generated packages and their dependencies."""
    if get_package_name_mode(vinca_conf) is PackageNameMode.LEGACY:
        return distro.get_legacy_package_prefix()
    return distro.get_package_prefix()


def get_new_package_name(pkg_shortname, distro):
    """Return the ROS-major-version package name."""
    return f"{distro.get_package_prefix()}-{pkg_shortname.replace('_', '-')}"


def get_legacy_package_name(pkg_shortname, distro):
    """Return the distro-qualified package name."""
    return f"{distro.get_legacy_package_prefix()}-{pkg_shortname.replace('_', '-')}"


def get_package_name(pkg_shortname, distro, vinca_conf):
    """Return the primary generated name for a ROS package."""
    if get_package_name_mode(vinca_conf) is PackageNameMode.LEGACY:
        return get_legacy_package_name(pkg_shortname, distro)
    return get_new_package_name(pkg_shortname, distro)


def normalize_package_dependency(dependency, distro, vinca_conf):
    """Rewrite current-distro legacy dependency names for the selected mode."""
    if isinstance(dependency, dict):
        return {
            key: normalize_package_dependency(value, distro, vinca_conf)
            for key, value in dependency.items()
        }
    if isinstance(dependency, list):
        return [
            normalize_package_dependency(value, distro, vinca_conf)
            for value in dependency
        ]
    if not isinstance(dependency, str):
        return dependency

    legacy_prefix = f"{distro.get_legacy_package_prefix()}-"
    if not dependency.startswith(legacy_prefix):
        return dependency

    package_prefix = f"{get_package_prefix(distro, vinca_conf)}-"
    return package_prefix + dependency[len(legacy_prefix) :]


def generate_legacy_compatibility_output(output, pkg_shortname, distro, vinca_conf):
    """Create an old-name package that depends on the renamed package, if requested."""
    if get_package_name_mode(vinca_conf) is not PackageNameMode.BOTH:
        return None

    canonical_name = get_new_package_name(pkg_shortname, distro)
    if output["package"]["name"] != canonical_name:
        return None

    legacy_name = get_legacy_package_name(pkg_shortname, distro)
    version = output["package"]["version"]
    return {
        "package": {"name": legacy_name, "version": version},
        "build": {"script": ""},
        "requirements": {"run": [f"{canonical_name} =={version}"]},
        "about": {"summary": f"Compatibility package for {canonical_name}"},
    }


def is_legacy_compatibility_output(output, distro, vinca_conf):
    """Return whether an output is one of the generated compatibility packages."""
    if get_package_name_mode(vinca_conf) is not PackageNameMode.BOTH:
        return False

    legacy_prefix_getter = getattr(distro, "get_legacy_package_prefix", None)
    if legacy_prefix_getter is None:
        return False

    name = output["package"]["name"]
    legacy_prefix = f"{legacy_prefix_getter()}-"
    if not name.startswith(legacy_prefix):
        return False

    shortname = name[len(legacy_prefix) :]
    canonical_name = get_new_package_name(shortname, distro)
    version = output["package"]["version"]
    return output.get("requirements", {}).get("run") == [
        f"{canonical_name} =={version}"
    ]
