import pytest


def pytest_collection_modifyitems(config, items):
    """
    Modify test items to add timeout and grouping for tests that use managed_processes.
    This helps prevent hanging tests and conflicts between tests that manage processes.
    """
    for item in items:
        # Add timeout to all sunnypilot tests to prevent hanging
        if "timeout" not in [mark.name for mark in item.iter_markers()]:
            item.add_marker(pytest.mark.timeout(120))  # 2 minute timeout per test

        # Group tests that manage external processes to run in the same worker to avoid conflicts
        if any(keyword in item.nodeid for keyword in [
            'test_sensord', 'test_locationd', 'test_modeld', 'test_pandad',
            'test_feedbackd', 'test_pigeond'
        ]):
            # Use xdist group to ensure these tests don't run in parallel
            # This prevents conflicts with managed processes
            item.add_marker(pytest.mark.xdist_group("managed_processes"))