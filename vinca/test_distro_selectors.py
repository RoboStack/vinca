from vinca.v1_selectors import evaluate_distro_selectors

TEST = {
    "tests": [
        {
            "script": [
                {
                    "if": 'ros_distro in ["humble", "jazzy"]',
                    "then": [{"if": "osx", "then": "old-osx"}],
                    "else": [{"if": "osx", "then": "new-osx"}],
                },
                {"if": 'ros_distro == "humble"', "then": ["humble-only"]},
                {"if": "linux", "then": "linux-cmd"},
            ]
        }
    ]
}


def test_distro_selectors_are_resolved_platform_ones_kept():
    humble = evaluate_distro_selectors(TEST, ros_distro="humble")
    assert humble["tests"][0]["script"] == [
        {"if": "osx", "then": "old-osx"},
        "humble-only",
        {"if": "linux", "then": "linux-cmd"},
    ]

    rolling = evaluate_distro_selectors(TEST, ros_distro="rolling")
    assert rolling["tests"][0]["script"] == [
        {"if": "osx", "then": "new-osx"},
        {"if": "linux", "then": "linux-cmd"},
    ]
