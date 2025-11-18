#!/usr/bin/env python3
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
        print("\nUI System stopped by user")
    except Exception as e:
        print(f"Error starting UI system: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
