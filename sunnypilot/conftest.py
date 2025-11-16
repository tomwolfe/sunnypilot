import pytest


def pytest_collection_modifyitems(config, items):
    """
    Modify test items to add timeout for sunnypilot tests to prevent hanging.
    """
    for item in items:
        # Add timeout to all sunnypilot tests to prevent hanging
        if "timeout" not in [mark.name for mark in item.iter_markers()]:
            item.add_marker(pytest.mark.timeout(120))  # 2 minute timeout per test