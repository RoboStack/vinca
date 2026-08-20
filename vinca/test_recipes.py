import pytest

from vinca.recipes import _dummy_constraint


def test_dummy_constraint_uses_pin_depth_and_override_version():
    version, constraint = _dummy_constraint(
        {
            "dep_name": "vendor-library",
            "upper_bound": "x.x",
            "override_version": "2.4.1",
        },
        "1.0.0",
        "demo_vendor",
    )

    assert version == "2.4.1"
    assert constraint == "vendor-library >=2.4.1, <2.5.0a0"


@pytest.mark.parametrize(
    ("config", "message"),
    [
        ({"upper_bound": "x"}, "dep_name"),
        ({"dep_name": "vendor-library"}, "upper_bound.*max_pin"),
    ],
)
def test_dummy_constraint_validates_required_settings(config, message):
    with pytest.raises(RuntimeError, match=message):
        _dummy_constraint(config, "1.0.0", "demo_vendor")
