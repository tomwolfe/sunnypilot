import pytest

def _has_panda_support():
    try:
        # Only try import, don't instantiate hardware
        import panda  # noqa: F401
        return True
    except Exception:
        return False

def pytest_collection_modifyitems(config, items):
    has_panda = _has_panda_support()
    if not has_panda:
        for item in items:
            # skip tests in this directory (pandad tests)
            if "selfdrive/pandad/tests" in str(item.fspath):
                item.add_marker(pytest.mark.skip(reason="No panda hardware in CI"))