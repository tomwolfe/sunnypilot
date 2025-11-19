import pytest
from unittest.mock import patch, MagicMock
import time
from system.sensord.system_health_monitoring import SystemHealthMonitor, SystemHealthReport, get_system_health

# Mock psutil
@pytest.fixture
def mock_psutil():
    with patch('system.sensord.system_health_monitoring.psutil') as mock:
        yield mock

def test_system_health_monitor_healthy(mock_psutil):
    # Configure psutil mocks for a healthy system
    mock_psutil.cpu_percent.return_value = 10.0
    mock_psutil.virtual_memory.return_value = MagicMock(percent=30.0)
    mock_psutil.disk_usage.return_value = MagicMock(percent=50.0)
    mock_psutil.sensors_temperatures.return_value = {'cpu_thermal': [MagicMock(current=50.0)]}

    monitor = SystemHealthMonitor()
    report = monitor.get_system_health_report()

    assert report.system_status == "HEALTHY"
    assert not report.warnings
    assert not report.critical_issues
    assert report.cpu_usage == 10.0
    assert report.memory_usage == 30.0
    assert report.disk_usage == 50.0
    assert report.thermal_sensors['cpu_thermal'] == 50.0
    assert isinstance(report.timestamp, float)

def test_system_health_monitor_warnings(mock_psutil):
    # Configure psutil mocks for warnings
    mock_psutil.cpu_percent.return_value = 80.0 # Warning
    mock_psutil.virtual_memory.return_value = MagicMock(percent=88.0) # Warning
    mock_psutil.disk_usage.return_value = MagicMock(percent=92.0) # Warning
    mock_psutil.sensors_temperatures.side_effect = AttributeError # Simulate platform without sensors

    monitor = SystemHealthMonitor()
    report = monitor.get_system_health_report()

    assert report.system_status == "WARNING"
    assert len(report.warnings) == 3
    assert "WARNING: High CPU usage: 80.0%" in report.warnings
    assert "WARNING: High memory usage: 88.0%" in report.warnings
    assert "WARNING: High disk usage: 92.0%" in report.warnings
    assert not report.critical_issues
    assert report.thermal_sensors == {"not_available": -1}

def test_system_health_monitor_critical_issues(mock_psutil):
    # Configure psutil mocks for critical issues
    mock_psutil.cpu_percent.return_value = 98.0 # Critical
    mock_psutil.virtual_memory.return_value = MagicMock(percent=96.0) # Critical
    mock_psutil.disk_usage.return_value = MagicMock(percent=99.0) # Critical
    mock_psutil.sensors_temperatures.return_value = {'chip_temp': [MagicMock(current=95.0)]}

    monitor = SystemHealthMonitor()
    report = monitor.get_system_health_report()

    assert report.system_status == "CRITICAL"
    assert len(report.critical_issues) == 3
    assert "CRITICAL: CPU usage is extremely high: 98.0%" in report.critical_issues
    assert "CRITICAL: Memory usage is extremely high: 96.0%" in report.critical_issues
    assert "CRITICAL: Disk usage is extremely high: 99.0%" in report.critical_issues
    assert not report.warnings
    assert report.thermal_sensors['chip_temp'] == 95.0

def test_get_system_health_function_wrapper(mock_psutil): # Renamed to avoid confusion with the method
    mock_psutil.cpu_percent.return_value = 5.0
    mock_psutil.virtual_memory.return_value = MagicMock(percent=15.0)
    mock_psutil.disk_usage.return_value = MagicMock(percent=25.0)
    mock_psutil.sensors_temperatures.return_value = {}

    report = get_system_health() # Call the standalone function
    # It should return a SystemHealthReport instance
    assert isinstance(report, SystemHealthReport)
    assert report.system_status == "HEALTHY"
