#!/usr/bin/env python3
"""
Circuit Breaker Manager for Controls.

This module handles circuit breaking functionality to prevent cascading failures.
"""

import time
from typing import Dict, Any

from openpilot.common.swaglog import cloudlog


class CircuitBreakerManager:
  """Manages circuit breakers to prevent cascading failures in control systems."""
  
  def __init__(self):
    self._circuit_breakers = self._init_circuit_breakers()

  def _init_circuit_breakers(self) -> Dict[str, Any]:
    """Initialize circuit breakers to prevent cascading failures."""
    return {
      'adaptive_gains': {
        'enabled': True,
        'error_count': 0,
        'max_errors': 3,  # Reduced from 5 to prevent potential abuse
        'last_error_time': 0,
        'last_error_reset_time': 0,  # Time when error count was last reset
        'cooldown_period': 10.0,  # Increased cooldown to 10 seconds to allow for root cause analysis
        'root_cause_analysis': [],  # Store recent error patterns for analysis
      },
      'radar_camera_fusion': {
        'enabled': True,
        'error_count': 0,
        'max_errors': 3,
        'last_error_time': 0,
        'last_error_reset_time': 0,
        'cooldown_period': 15.0,  # Increased for fusion which is critical
        'root_cause_analysis': [],  # Store recent error patterns for analysis
      },
      'vision_model_optimization': {
        'enabled': True,
        'error_count': 0,
        'max_errors': 5,  # Reduced from 10 to prevent abuse
        'last_error_time': 0,
        'last_error_reset_time': 0,
        'cooldown_period': 45.0,  # Increased to 45 seconds for vision model
        'root_cause_analysis': [],  # Store recent error patterns for analysis
      },
    }

  def check_circuit_breaker(self, breaker_name: str) -> bool:
    """Check if a circuit breaker is enabled and handle cooldown periods."""
    cb = self._circuit_breakers[breaker_name]

    # Check if we're in cooldown period after an error
    current_time = time.monotonic()
    if not cb['enabled']:
      # Calculate time since the last error occurred
      time_since_last_error = current_time - cb['last_error_time']

      # Check if cooldown period has passed since the last error
      if time_since_last_error > cb['cooldown_period']:
        # For the breaker to reset, we need to have waited not just the cooldown period,
        # but also a "stable" period (half the cooldown period) since the last error
        # This prevents the breaker from resetting too soon after the last error,
        # requiring a period of stability before re-enabling the feature
        required_total_wait_time = cb['cooldown_period'] + (cb['cooldown_period'] / 2)
        if time_since_last_error >= required_total_wait_time:
          cb['enabled'] = True
          cb['error_count'] = 0  # Reset error count on successful recovery
          cb['last_error_reset_time'] = current_time
          cloudlog.info(f"Circuit breaker {breaker_name} reset after cooldown and stable period")
        else:
          return False  # Not yet ready to reset - need to wait for stable period after the error
      else:
        return False  # Still in cooldown, circuit breaker is disabled

    return cb['enabled']

  def trigger_circuit_breaker(self, breaker_name: str, error_msg: str, error_type: str = None) -> None:
    """Trigger a circuit breaker due to an error with enhanced root cause tracking."""
    cb = self._circuit_breakers[breaker_name]
    cb['error_count'] += 1
    cb['enabled'] = False
    cb['last_error_time'] = time.monotonic()

    # Add error to root cause analysis
    if error_type is None:
      error_type = "unknown"
    cb['root_cause_analysis'].append(
      {'timestamp': cb['last_error_time'], 'error_type': error_type, 'error_msg': error_msg, 'error_count_at_time': cb['error_count']}
    )

    # Keep only the most recent 10 errors for analysis
    if len(cb['root_cause_analysis']) > 10:
      cb['root_cause_analysis'] = cb['root_cause_analysis'][-10:]

    cloudlog.error(
      f"Circuit breaker {breaker_name} triggered due to error: {error_msg} "
      + f"(type: {error_type}). Error count: {cb['error_count']}/{cb['max_errors']} "
      + f"at time {cb['last_error_time']:.2f}"
    )

    # Log detailed root cause analysis if we have multiple errors
    if cb['error_count'] > 1 and len(cb['root_cause_analysis']) > 1:
      # Analyze error patterns
      recent_errors = cb['root_cause_analysis'][-5:]  # Look at last 5 errors
      error_types = [e['error_type'] for e in recent_errors]
      time_diffs = [recent_errors[i + 1]['timestamp'] - recent_errors[i]['timestamp'] for i in range(len(recent_errors) - 1)] if len(recent_errors) > 1 else []

      if len(set(error_types)) == 1:
        # All recent errors are the same type - potential systematic issue
        cloudlog.warning(f"Circuit breaker {breaker_name}: Repeated {error_types[0]} errors detected - possible systematic issue")
      if time_diffs and all(td < 2.0 for td in time_diffs):
        # Errors happening in rapid succession - potential cascade
        cloudlog.warning(f"Circuit breaker {breaker_name}: Rapid error sequence detected - possible cascade failure")

    if cb['error_count'] >= cb['max_errors']:
      cloudlog.critical(
        f"Circuit breaker {breaker_name} permanently disabled due to excessive errors. "
        + f"Root cause analysis: {cb['root_cause_analysis'][-3:] if cb['root_cause_analysis'] else []}"
      )