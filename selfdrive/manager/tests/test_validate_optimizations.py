import pytest
from unittest.mock import patch, MagicMock
import time
from selfdrive.manager.validate_optimizations import validate_hardware_constraints, check_hardware_status
from openpilot.common.swaglog import cloudlog # Import for mocking

# Mock cloudlog used in the module
@pytest.fixture(autouse=True)
def mock_cloudlog():
    with patch('selfdrive.manager.validate_optimizations.cloudlog') as mock_log:
        yield mock_log

# Mock psutil
@pytest.fixture
def mock_psutil():
    with patch('selfdrive.manager.validate_optimizations.psutil') as mock:
        yield mock

# Mock cereal.messaging
@pytest.fixture
def mock_messaging():
    with patch('selfdrive.manager.validate_optimizations.messaging') as mock:
        yield mock

def test_validate_hardware_constraints_all_valid(mock_psutil, mock_messaging):
    # Mock psutil for valid conditions
    mock_psutil.virtual_memory.return_value = MagicMock(used=1.0 * (1024**3)) # 1.0 GB < 1.4 GB
    mock_psutil.cpu_percent.return_value = 5.0 # 5% < 10%

    # Mock messaging for valid latency
    mock_sm = MagicMock()
    mock_sm.updated = {'modelV2': True, 'controlsState': True}
    mock_sm.logMonoTime = {'modelV2': 1000 * 1e9, 'controlsState': 1070 * 1e9} # 70 ms latency < 80 ms
    mock_messaging.SubMaster.return_value = mock_sm

    results = validate_hardware_constraints()

    assert results['ram_valid'] is True
    assert results['cpu_valid'] is True
    assert results['latency_valid'] is True
    assert results['all_valid'] is True
    assert results['ram_usage_gb'] == 1.0
    assert results['cpu_percent'] == 5.0
    assert results['latency_ms'] == 70.0

def test_validate_hardware_constraints_ram_invalid(mock_psutil, mock_messaging):
    # Mock psutil for invalid RAM
    mock_psutil.virtual_memory.return_value = MagicMock(used=1.5 * (1024**3)) # 1.5 GB > 1.4 GB
    mock_psutil.cpu_percent.return_value = 5.0

    # Mock messaging for valid latency
    mock_sm = MagicMock()
    mock_sm.updated = {'modelV2': True, 'controlsState': True}
    mock_sm.logMonoTime = {'modelV2': 1000 * 1e9, 'controlsState': 1070 * 1e9}
    mock_messaging.SubMaster.return_value = mock_sm

    results = validate_hardware_constraints()

    assert results['ram_valid'] is False
    assert results['cpu_valid'] is True
    assert results['latency_valid'] is True
    assert results['all_valid'] is False
    assert results['ram_usage_gb'] == 1.5

def test_validate_hardware_constraints_cpu_invalid(mock_psutil, mock_messaging):
    # Mock psutil for invalid CPU
    mock_psutil.virtual_memory.return_value = MagicMock(used=1.0 * (1024**3))
    mock_psutil.cpu_percent.return_value = 12.0 # 12% > 10%

    # Mock messaging for valid latency
    mock_sm = MagicMock()
    mock_sm.updated = {'modelV2': True, 'controlsState': True}
    mock_sm.logMonoTime = {'modelV2': 1000 * 1e9, 'controlsState': 1070 * 1e9}
    mock_messaging.SubMaster.return_value = mock_sm

    results = validate_hardware_constraints()

    assert results['ram_valid'] is True
    assert results['cpu_valid'] is False
    assert results['latency_valid'] is True
    assert results['all_valid'] is False
    assert results['cpu_percent'] == 12.0

def test_validate_hardware_constraints_latency_invalid(mock_psutil, mock_messaging):
    # Mock psutil for valid conditions
    mock_psutil.virtual_memory.return_value = MagicMock(used=1.0 * (1024**3))
    mock_psutil.cpu_percent.return_value = 5.0

    # Mock messaging for invalid latency
    mock_sm = MagicMock()
    mock_sm.updated = {'modelV2': True, 'controlsState': True}
    mock_sm.logMonoTime = {'modelV2': 1000 * 1e9, 'controlsState': 1090 * 1e9} # 90 ms latency > 80 ms
    mock_messaging.SubMaster.return_value = mock_sm

    results = validate_hardware_constraints()

    assert results['ram_valid'] is True
    assert results['cpu_valid'] is True
    assert results['latency_valid'] is False
    assert results['all_valid'] is False
    assert results['latency_ms'] == 90.0

def test_validate_hardware_constraints_no_messaging_update(mock_psutil, mock_messaging):
    # Mock psutil for valid conditions
    mock_psutil.virtual_memory.return_value = MagicMock(used=1.0 * (1024**3))
    mock_psutil.cpu_percent.return_value = 5.0

    # Mock messaging not updated
    mock_sm = MagicMock()
    mock_sm.updated = {'modelV2': False, 'controlsState': False}
    mock_messaging.SubMaster.return_value = mock_sm

    results = validate_hardware_constraints()

    assert results['ram_valid'] is True
    assert results['cpu_valid'] is True
    assert results['latency_valid'] is True # Assumed valid if cannot measure
    assert results['all_valid'] is True
    assert results['latency_ms'] == 0.0

def test_check_hardware_status_function(mock_psutil, mock_messaging):
    # Configure psutil for valid conditions
    mock_psutil.virtual_memory.return_value = MagicMock(used=1.0 * (1024**3))
    mock_psutil.cpu_percent.return_value = 5.0

    # Configure messaging for valid latency
    mock_sm = MagicMock()
    mock_sm.updated = {'modelV2': True, 'controlsState': True}
    mock_sm.logMonoTime = {'modelV2': 1000 * 1e9, 'controlsState': 1070 * 1e9}
    mock_messaging.SubMaster.return_value = mock_sm

    results = check_hardware_status()
    assert results['all_valid'] is True
