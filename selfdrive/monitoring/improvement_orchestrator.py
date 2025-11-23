"""
Safe Improvement Framework for Autonomous Driving
Main orchestrator with safety validation for parameter changes

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Import the safe improvement orchestrator class
# This provides all the safety validation and human-in-the-loop protections
from .safe_improvement_orchestrator import SafeImprovementOrchestrator

# For backward compatibility, maintain the original class name
ImprovementPlanOrchestrator = SafeImprovementOrchestrator

__all__ = ['ImprovementPlanOrchestrator', 'SafeImprovementOrchestrator']