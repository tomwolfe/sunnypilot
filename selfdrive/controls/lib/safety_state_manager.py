#!/usr/bin/env python3
"""
Thread-safe safety state manager for autonomous driving systems
Replaces global state variables with proper encapsulation and thread safety
"""

import threading
from typing import Dict, Any


class SafetyStateManager:
    """
    Thread-safe class to manage safety-critical state variables across function calls
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._prev_curvature = 0.0
        self._error_count = 0
        
    def update_error(self, current_curvature: float):
        """
        Update error state with new curvature value and increment error count
        """
        with self._lock:
            self._prev_curvature = current_curvature
            self._error_count += 1

    def reset_errors(self):
        """
        Reset error state when operations succeed
        """
        with self._lock:
            self._error_count = 0

    def get_fallback_curvature(self) -> float:
        """
        Get fallback curvature based on current error state
        """
        with self._lock:
            if self._error_count >= 8:
                # Disengage system after max failures - return neutral curvature
                return 0.0
            elif self._error_count >= 6:  # Apply 90% conservative factor after 6 errors
                return self._prev_curvature * ERROR_HANDLING['degradation_factors'][6]
            elif self._error_count >= 3:  # Apply 70% conservative factor after 3 errors
                return self._prev_curvature * ERROR_HANDLING['degradation_factors'][3]
            else:  # Use historical data for first few errors
                return self._prev_curvature

    def get_error_count(self) -> int:
        """
        Get current error count for monitoring
        """
        with self._lock:
            return self._error_count
            
    def set_error_count(self, count: int):
        """
        Set error count (for testing purposes)
        """
        with self._lock:
            self._error_count = max(0, count)  # Ensure non-negative


class GlobalSafetyStateManager:
    """
    Singleton class to provide global access to safety state manager
    """
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._safety_state = SafetyStateManager()
        return cls._instance

    def get_state_manager(self):
        """
        Get the safety state manager instance
        """
        return self._safety_state


# Error handling policy constants
ERROR_HANDLING = {
    'max_consecutive_failures': 8,
    'failure_degradation': {
        1: 'use_historical_data',
        3: 'apply_70_percent_conservative_factor',
        6: 'apply_90_percent_conservative_factor',
        8: 'disengage'
    },
    'degradation_factors': {
        3: 0.7,  # 70% conservative factor after 3 failures
        6: 0.9   # 90% conservative factor after 6 failures
    }
}


def get_global_safety_state_manager():
    """
    Get the global safety state manager instance
    """
    return GlobalSafetyStateManager().get_state_manager()