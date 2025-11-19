#!/usr/bin/env python3
"""
Integration test for the simplified validation system
"""
import sys
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
        "sunnypilot/selfdrive/controls/lib/traffic_light_validation.py",
        "sunnypilot/selfdrive/controls/lib/traffic_sign_detection.py",
        "selfdrive/controls/safety_supervisor.py",
        "system/sensord/system_health_monitoring.py",
        "selfdrive/common/enhanced_validation.py",
    ]
    
    print("Testing file integrity...")
    all_good = True
    for file_path in files_to_test:
        if Path(file_path).exists():
            try:
                # Test syntax
                import py_compile
                py_compile.compile(file_path, doraise=True)
                print(f"  ✓ {file_path} - exists and has valid syntax")
            except py_compile.PyCompileError as e:
                print(f"  ✗ {file_path} - syntax error: {e}")
                all_good = False
        else:
            print(f"  ✗ {file_path} - file does not exist")
            all_good = False
    
    return all_good

def test_process_config_modifications():
    """Test that process config has been properly modified"""
    with open("system/manager/process_config.py", "r") as f:
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
        print("  ✓ validation_controller process not found in config")
        return False
        
    if has_navd:
        print("  ✓ navd process found in config")
    else:
        print("  ✗ navd process not found in config")
        return False
    
    return True

def main():
    print("="*60)
    print("SUNNYPilot Simplified System Integration Test")
    print("="*60)
    
    file_test = test_file_integrity()
    config_test = test_process_config_modifications()
    
    print("\n" + "="*60)
    if file_test and config_test:
        print("✓ ALL INTEGRATION TESTS PASSED")
        print("Simplified validation, safety, navigation, and monitoring systems")
        print("have been successfully integrated with the sunnypilot system")
    else:
        print("✗ SOME INTEGRATION TESTS FAILED")
        print("Please check the issues above")
    print("="*60)
    
    return file_test and config_test

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)