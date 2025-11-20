#!/usr/bin/env python3
"""
Configuration system for validation parameters in Sunnypilot
Provides configurable parameters to replace hardcoded values
"""
from dataclasses import dataclass
from typing import Dict, Any
import json


@dataclass
class ValidationConfig:
    """Configuration for validation systems"""
    # Confidence thresholds
    confidence_threshold: float = 0.7
    min_confidence_for_engagement: float = 0.6
    
    # Environment complexity limits
    max_environment_complexity: float = 0.8
    
    # Temporal consistency requirements  
    min_temporal_consistency: float = 0.3
    
    # Velocity limits for engagement
    max_engagement_velocity: float = 60.0  # m/s (about 134 mph)
    min_engagement_velocity: float = 0.1   # m/s
    
    # Safety factors
    environment_complexity_factor: float = 0.3
    safety_environment_factor: float = 0.2
    
    # Performance parameters
    max_velocity_history: int = 50
    max_previous_states: int = 20
    validation_frequency: float = 20.0  # Hz
    
    # Panda safety limits
    max_panda_rx_invalid: int = 20
    min_lead_distance: float = 50.0  # meters for complexity calculation
    
    # Distance factors for lead confidence
    close_lead_distance: float = 10.0  # meters
    far_lead_distance: float = 100.0   # meters
    
    # Speed threshold for complexity calculation
    high_speed_threshold: float = 25.0  # m/s


def load_validation_config(config_path: str = None) -> ValidationConfig:
    """Load validation configuration from file or return defaults"""
    if config_path and _config_file_exists(config_path):
        try:
            with open(config_path, 'r') as f:
                config_dict = json.load(f)
            return ValidationConfig(**config_dict)
        except Exception:
            # If config file is invalid, return defaults
            pass
    
    return ValidationConfig()


def _config_file_exists(path: str) -> bool:
    """Check if config file exists"""
    try:
        import os
        return os.path.exists(path)
    except:
        return False


# Global configuration instance
_validation_config = load_validation_config()


def get_validation_config() -> ValidationConfig:
    """Get the current validation configuration"""
    return _validation_config


def update_validation_config(new_config: ValidationConfig):
    """Update the validation configuration"""
    global _validation_config
    _validation_config = new_config