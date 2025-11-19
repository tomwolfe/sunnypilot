#!/usr/bin/env python3
"""
Simple Performance Validation for Sunnypilot
Minimal validation that essential components are present
"""
import os
from pathlib import Path


def validate_essential_files():
  """Validate that essential files exist"""
  essential_files = [
    "selfdrive/common/enhanced_validation.py",
    "selfdrive/controls/safety_supervisor.py",
    "selfdrive/common/validation_publisher.py",
    "selfdrive/monitoring/validation_controller.py",
    "sunnypilot/navd/navd.py",
    "sunnypilot/selfdrive/controls/lib/traffic_light_validation.py",
    "system/sensord/system_health_monitoring.py",
  ]
  
  all_present = True
  for filepath in essential_files:
    path = Path(filepath)
    exists = path.exists()
    status = "✓" if exists else "✗"
    print(f"  {status} {filepath}")
    if not exists:
      all_present = False
  
  return all_present


def main():
  """Run simple validation"""
  print("="*50)
  print("SUNNYPilot Essential Components Validation")
  print("="*50)
  
  all_valid = validate_essential_files()
  
  print("\n" + "="*50)
  if all_valid:
    print("✓ ALL ESSENTIAL COMPONENTS PRESENT")
    print("Sunnypilot system has essential validation components")
  else:
    print("✗ SOME ESSENTIAL COMPONENTS MISSING")
    print("Please check the missing files")
  print("="*50)
  
  return all_valid


if __name__ == "__main__":
  success = main()
  exit(0 if success else 1)