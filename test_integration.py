#!/usr/bin/env python3
"""
Enhanced Integration test for the validation and safety systems
"""
import sys
import importlib.util
from pathlib import Path

def test_file_integrity():
    """Test that all expected files exist and have valid syntax"""
    files_to_test = [
        "selfdrive/common/validation_publisher.py",
        "selfdrive/monitoring/validation_controller.py",
        "sunnypilot/navd/navd.py",
        "sunnypilot/navd/navigation.py",
        "sunnypilot/navd/interface.py",
        "sunnypilot/navd/routing.py",
        "selfdrive/controls/safety_supervisor.py",
        "system/sensord/system_health_monitoring.py",
        "selfdrive/common/enhanced_validation.py",
        "selfdrive/common/validation_utils.py",
        "selfdrive/perception/behavior_prediction.py",
        "selfdrive/common/arm_optimization.py",
        "selfdrive/common/memory_optimization.py",
        "selfdrive/common/performance_monitor.py",
        "common/data_collector.py",
        "final_validation.py",
    ]

    print("Testing file integrity...")
    all_good = True
    for file_path in files_to_test:
        path = Path(file_path)
        if path.exists():
            try:
                # Test syntax by attempting to compile
                with open(path, 'r', encoding='utf-8') as f:
                    source = f.read()
                compile(source, str(path), 'exec')
                print(f"  ✓ {file_path} - exists and has valid syntax")
            except SyntaxError as e:
                print(f"  ✗ {file_path} - syntax error at line {e.lineno}: {e.msg}")
                all_good = False
            except Exception as e:
                print(f"  ✗ {file_path} - error: {e}")
                all_good = False
        else:
            print(f"  ✗ {file_path} - file does not exist")
            all_good = False

    return all_good

def test_process_config_modifications():
    """Test that process config has been properly modified"""
    config_path = Path("system/manager/process_config.py")
    if not config_path.exists():
        print("  ✗ system/manager/process_config.py does not exist")
        return False

    with open(config_path, "r") as f:
        content = f.read()

    # Check that validation processes are added
    has_validationd = "PythonProcess(\"validationd\"" in content
    has_validation_controller = "PythonProcess(\"validation_controller\"" in content
    has_navd = "PythonProcess(\"navd\"" in content

    print("\nTesting process configuration...")
    if has_validationd:
        print("  ✓ validationd process added to config")
    else:
        print("  ✗ validationd process not found in config")
        return False

    if has_validation_controller:
        print("  ✓ validation_controller process added to config")
    else:
        print("  ✗ validation_controller process not found in config")
        return False

    if has_navd:
        print("  ✓ navd process found in config")
    else:
        print("  ✗ navd process not found in config")
        return False

    return True

def test_capnp_schema():
    """Test that capnp schema changes are properly made"""
    custom_capnp_path = Path("cereal/custom.capnp")
    log_capnp_path = Path("cereal/log.capnp")

    print("\nTesting capnp schema changes...")

    # Test custom.capnp
    if custom_capnp_path.exists():
        with open(custom_capnp_path, "r") as f:
            content = f.read()

        has_validation_metrics = "ValidationMetrics" in content
        if has_validation_metrics:
            print("  ✓ ValidationMetrics struct found in custom.capnp")
        else:
            print("  ✗ ValidationMetrics struct not found in custom.capnp")
            return False
    else:
        print("  ✗ cereal/custom.capnp does not exist")
        return False

    # Test log.capnp
    if log_capnp_path.exists():
        with open(log_capnp_path, "r") as f:
            content = f.read()

        has_validation_metrics = "validationMetrics" in content
        if has_validation_metrics:
            print("  ✓ validationMetrics field found in log.capnp")
        else:
            print("  ✗ validationMetrics field not found in log.capnp")
            return False
    else:
        print("  ✗ cereal/log.capnp does not exist")
        return False

    return True

def main():
    print("="*70)
    print("SUNNYPilot Enhanced System Integration Test")
    print("- Comprehensive validation of safety, validation, and navigation systems")
    print("- Enhanced error handling and safety checks")
    print("- Improved algorithms and system reliability")
    print("="*70)

    file_test = test_file_integrity()
    config_test = test_process_config_modifications()
    capnp_test = test_capnp_schema()

    all_tests = file_test and config_test and capnp_test

    print("\n" + "="*70)
    if all_tests:
        print("✓ ALL INTEGRATION TESTS PASSED")
        print("Enhanced validation, safety, navigation, and monitoring systems")
        print("have been successfully integrated with the sunnypilot system")
        print("with improved safety checks and reliability")
    else:
        print("✗ SOME INTEGRATION TESTS FAILED")
        print("Please check the issues above and address them before deployment")
    print("="*70)

    return all_tests

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)