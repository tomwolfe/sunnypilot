"""
Refactored Safety Monitoring for sunnypilot - Main orchestrator

This version addresses the complexity concerns by using separated modules
while maintaining all safety functionality.

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Import the refactored SafetyMonitor class
# This maintains backward compatibility while using the new architecture
from .refactored.safety_monitor_refactored import SafetyMonitor

__all__ = ['SafetyMonitor']