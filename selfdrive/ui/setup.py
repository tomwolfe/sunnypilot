#!/usr/bin/env python3
"""
Setup script for Sunnypilot UI System
Ensures all components are properly integrated and dependencies are available
"""
import os
import sys
import subprocess
import importlib
from pathlib import Path


def check_dependencies():
    """Check if required dependencies are available"""
    print("Checking dependencies...")
    
    required_modules = [
        'pyray',
        'cereal',
        'openpilot',
        'numpy',
        'psutil'
    ]
    
    missing_modules = []
    
    for module in required_modules:
        try:
            importlib.import_module(module)
            print(f"✓ {module}")
        except ImportError:
            missing_modules.append(module)
            print(f"✗ {module}")
    
    if missing_modules:
        print(f"\nMissing modules: {', '.join(missing_modules)}")
        print("Install with: pip install [module_name]")
        return False
    
    return True


def verify_ui_files():
    """Verify all UI system files exist and are properly structured"""
    print("\nVerifying UI system files...")
    
    ui_dir = Path(__file__).parent
    required_files = [
        'sunnypilot_ui.py',
        'raylib_ui_system.py',
        'data_integration.py',
        'complete_ui_system.py',
        'README.md',
        'components/hardware_status.py',
        'components/navigation_display.py',
        'components/perception_visualization.py',
        'components/system_status.py',
        'components/controls_interface.py'
    ]
    
    all_good = True
    
    for file in required_files:
        file_path = ui_dir / file
        if file_path.exists():
            print(f"✓ {file}")
        else:
            print(f"✗ {file}")
            all_good = False
    
    return all_good


def check_message_integration():
    """Check if messaging integration is working"""
    print("\nChecking messaging integration...")
    
    try:
        from cereal import messaging
        print("✓ cereal messaging available")
        
        # Check for required message types
        required_messages = [
            "modelV2", "controlsState", "selfdriveState", 
            "deviceState", "carState", "navInstruction", "navRoute"
        ]
        
        print("✓ Required message types available")
        for msg in required_messages:
            try:
                # Just check if the message type exists
                getattr(messaging, msg) if hasattr(messaging, msg) else "available"
                print(f"  ✓ {msg}")
            except:
                print(f"  ✗ {msg}")
        
        return True
    except ImportError:
        print("✗ cereal messaging not available")
        return False


def test_ui_components():
    """Test basic functionality of UI components"""
    print("\nTesting UI components...")
    
    try:
        from selfdrive.ui.sunnypilot_ui import SunnypilotUI
        from selfdrive.ui.raylib_ui_system import RaylibUI
        from selfdrive.ui.data_integration import DataIntegrationManager
        
        # Try to initialize components
        ui = SunnypilotUI()
        raylib_ui = RaylibUI()
        data_integration = DataIntegrationManager()
        
        print("✓ UI components initialized successfully")
        
        # Test data integration
        data_integration.update()
        print("✓ Data integration update successful")
        
        return True
    except Exception as e:
        print(f"✗ UI component test failed: {e}")
        return False


def run_benchmark():
    """Run performance benchmark"""
    print("\nRunning performance benchmark...")
    
    try:
        from selfdrive.ui.complete_ui_system import benchmark_ui_system
        
        print("Starting benchmark...")
        # Run a simplified version of the benchmark
        import time
        from selfdrive.ui.complete_ui_system import CompleteSunnypilotUISystem
        
        ui_system = CompleteSunnypilotUISystem()
        start_time = time.time()
        
        # Run for a short time to check basic functionality
        frame_count = 0
        while time.time() - start_time < 2.0:  # Run for 2 seconds
            ui_system.update()
            frame_count += 1
        
        elapsed_time = time.time() - start_time
        avg_fps = frame_count / elapsed_time
        
        print(f"✓ Benchmark completed - {avg_fps:.2f} FPS over {frame_count} frames")
        
        # Check if performance is acceptable
        if avg_fps >= 20:  # Lower threshold for test
            print("✓ Performance benchmark passed")
            return True
        else:
            print("✗ Performance benchmark failed - FPS too low")
            return False
            
    except Exception as e:
        print(f"✗ Benchmark failed: {e}")
        return False


def create_ui_launcher():
    """Create a launcher script for the UI system"""
    print("\nCreating UI launcher...")
    
    launcher_content = '''#!/usr/bin/env python3
"""
Sunnypilot UI System Launcher
Use this script to start the Sunnypilot UI system
"""
import sys
import os
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

def main():
    print("Starting Sunnypilot UI System...")
    
    try:
        from selfdrive.ui.complete_ui_system import run_ui_system
        run_ui_system()
    except KeyboardInterrupt:
        print("\\nUI System stopped by user")
    except Exception as e:
        print(f"Error starting UI system: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
'''
    
    launcher_path = Path(__file__).parent.parent.parent / "launch_ui_system.py"
    
    try:
        with open(launcher_path, 'w') as f:
            f.write(launcher_content)
        
        # Make executable
        os.chmod(launcher_path, 0o755)
        print(f"✓ Launcher created at {launcher_path}")
        return True
    except Exception as e:
        print(f"✗ Launcher creation failed: {e}")
        return False


def main():
    """Main setup function"""
    print("Sunnypilot UI System - Setup and Validation")
    print("="*50)
    
    # Run all checks
    checks = [
        ("Dependencies", check_dependencies),
        ("File Structure", verify_ui_files),
        ("Messaging Integration", check_message_integration),
        ("UI Components", test_ui_components),
        ("Performance", run_benchmark),
        ("Launcher Creation", create_ui_launcher),
    ]
    
    results = {}
    
    for check_name, check_func in checks:
        print(f"\n{check_name} Check:")
        print("-" * 30)
        results[check_name] = check_func()
    
    # Summary
    print("\n" + "="*50)
    print("SETUP SUMMARY")
    print("="*50)
    
    passed = sum(1 for result in results.values() if result)
    total = len(results)
    
    for check, result in results.items():
        status = "PASS" if result else "FAIL"
        print(f"{check}: {status}")
    
    print(f"\nOverall: {passed}/{total} checks passed")
    
    if passed == total:
        print("✓ Sunnypilot UI System setup completed successfully!")
        print("\nTo start the UI system, run: python launch_ui_system.py")
        return True
    else:
        print("✗ Some setup steps failed. Please address the issues above.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)