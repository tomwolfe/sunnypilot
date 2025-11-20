#!/usr/bin/env python3
"""
Simplified Configuration system for validation parameters in Sunnypilot
Provides basic configurable parameters
"""
from dataclasses import dataclass


@dataclass
class ValidationConfig:
    """Configuration for validation systems"""
    # Confidence thresholds
    confidence_threshold: float = 0.7
    # Performance parameters
    validation_frequency: float = 20.0  # Hz


def get_validation_config():
    """Get the current validation configuration"""
    return ValidationConfig()