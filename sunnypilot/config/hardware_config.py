"""
Hardware Configuration and Feature Management for Sunnypilot2

This module provides hardware-specific configuration and feature management
to ensure appropriate features are enabled based on hardware capabilities.
"""

import os
from typing import Dict, Any, List
from enum import Enum


class HardwareClass(Enum):
    """Different classes of hardware with varying capabilities."""
    SNAPDRAGON_845 = "snapdragon_845"
    COMMA_3 = "comma_3"
    COMMA_3X = "comma_3x"
    DEVELOPMENT = "development"


class HardwareConfig:
    """Configuration class for different hardware platforms."""

    def __init__(self, hardware_class: HardwareClass = None):
        self.hardware_class = hardware_class or self._detect_hardware()
        self.feature_config = self._get_feature_config(self.hardware_class)

    def _detect_hardware(self) -> HardwareClass:
        """Detect the hardware class automatically or from environment."""
        # Check environment variable first
        env_hardware = os.environ.get('HARDWARE_CLASS')
        if env_hardware:
            try:
                return HardwareClass(env_hardware)
            except ValueError:
                print(f"Warning: Unknown hardware class '{env_hardware}', defaulting to Snapdragon 845")
                return HardwareClass.SNAPDRAGON_845

        # In a real implementation, this would detect actual hardware
        # For now we'll default to Snapdragon 845 for safety
        return HardwareClass.SNAPDRAGON_845

    def _get_feature_config(self, hardware_class: HardwareClass) -> Dict[str, Any]:
        """Get feature configuration based on hardware class."""
        configs = {
            HardwareClass.SNAPDRAGON_845: {
                'max_compute_power': 'low',
                'available_features': [
                    'scene_detection',
                    'coordinated_control',
                    'basic_edge_case_detection',
                    'lightweight_integration'
                ],
                'disabled_features': [
                    'advanced_fusion',
                    'hierarchical_planning',
                    'optimized_mpc_control',
                    'self_learning_enhancement'
                ],
                'performance_target': {
                    'max_cycle_time_ms': 20,
                    'min_frame_rate': 15,  # fps
                    'memory_limit_mb': 512
                },
                'enabled': True
            },
            HardwareClass.COMMA_3: {
                'max_compute_power': 'medium',
                'available_features': [
                    'scene_detection',
                    'coordinated_control',
                    'basic_edge_case_detection',
                    'lightweight_integration',
                    'advanced_fusion'  # Limited version
                ],
                'disabled_features': [
                    'hierarchical_planning',
                    'optimized_mpc_control',
                    'self_learning_enhancement'
                ],
                'performance_target': {
                    'max_cycle_time_ms': 40,
                    'min_frame_rate': 20,  # fps
                    'memory_limit_mb': 1024
                },
                'enabled': True
            },
            HardwareClass.COMMA_3X: {
                'max_compute_power': 'high',
                'available_features': [
                    'scene_detection',
                    'coordinated_control',
                    'basic_edge_case_detection',
                    'lightweight_integration',
                    'advanced_fusion',
                    'hierarchical_planning',
                    'optimized_mpc_control'
                ],
                'disabled_features': [
                    'self_learning_enhancement'  # Still experimental
                ],
                'performance_target': {
                    'max_cycle_time_ms': 50,
                    'min_frame_rate': 20,  # fps
                    'memory_limit_mb': 2048
                },
                'enabled': True
            },
            HardwareClass.DEVELOPMENT: {
                'max_compute_power': 'unlimited',
                'available_features': [
                    'scene_detection',
                    'coordinated_control',
                    'basic_edge_case_detection',
                    'lightweight_integration',
                    'advanced_fusion',
                    'hierarchical_planning',
                    'optimized_mpc_control',
                    'self_learning_enhancement'
                ],
                'disabled_features': [],
                'performance_target': {
                    'max_cycle_time_ms': 100,
                    'min_frame_rate': 20,  # fps
                    'memory_limit_mb': 4096
                },
                'enabled': True
            }
        }

        return configs.get(hardware_class, configs[HardwareClass.SNAPDRAGON_845])

    def is_feature_available(self, feature: str) -> bool:
        """Check if a feature is available on the current hardware."""
        return feature in self.feature_config['available_features']

    def is_feature_disabled(self, feature: str) -> bool:
        """Check if a feature is disabled on the current hardware."""
        return feature in self.feature_config['disabled_features']

    def get_performance_targets(self) -> Dict[str, Any]:
        """Get performance targets for the current hardware."""
        return self.feature_config['performance_target']

    def get_hardware_specs(self) -> Dict[str, Any]:
        """Get specifications for the current hardware."""
        return {
            'hardware_class': self.hardware_class.value,
            'max_compute_power': self.feature_config['max_compute_power'],
            'available_features': self.feature_config['available_features'],
            'disabled_features': self.feature_config['disabled_features'],
            'enabled': self.feature_config['enabled']
        }


def get_active_hardware_config() -> HardwareConfig:
    """Get the active hardware configuration."""
    return HardwareConfig()


def validate_feature_availability(feature_name: str) -> bool:
    """
    Validate if a feature is available on the current hardware.
    
    Args:
        feature_name: Name of the feature to validate
        
    Returns:
        True if feature is available, False otherwise
    """
    config = get_active_hardware_config()
    return config.is_feature_available(feature_name)


def get_recommended_mode() -> str:
    """
    Get the recommended operational mode based on hardware.
    
    Returns:
        String indicating the recommended mode
    """
    config = get_active_hardware_config()
    if config.hardware_class == HardwareClass.SNAPDRAGON_845:
        return "lightweight"
    elif config.hardware_class in [HardwareClass.COMMA_3, HardwareClass.COMMA_3X]:
        return "balanced"
    else:
        return "advanced"


def print_hardware_info():
    """Print information about the current hardware configuration."""
    config = get_active_hardware_config()
    specs = config.get_hardware_specs()
    
    print("Hardware Configuration:")
    print(f"  Class: {specs['hardware_class']}")
    print(f"  Compute Power: {specs['max_compute_power']}")
    print(f"  Status: {'Enabled' if specs['enabled'] else 'Disabled'}")
    
    print(f"\nAvailable Features:")
    for feature in specs['available_features']:
        print(f"  - {feature}")
    
    if specs['disabled_features']:
        print(f"\nDisabled Features:")
        for feature in specs['disabled_features']:
            print(f"  - {feature}")
    
    targets = config.get_performance_targets()
    print(f"\nPerformance Targets:")
    print(f"  Max Cycle Time: {targets['max_cycle_time_ms']}ms")
    print(f"  Min Frame Rate: {targets['min_frame_rate']}fps")
    print(f"  Memory Limit: {targets['memory_limit_mb']}MB")


if __name__ == "__main__":
    print("Hardware Configuration and Feature Management")
    print("=" * 50)
    print_hardware_info()
    print("=" * 50)
    
    # Test feature validation
    print(f"\nFeature availability tests:")
    print(f"  Scene detection available: {validate_feature_availability('scene_detection')}")
    print(f"  Advanced fusion available: {validate_feature_availability('advanced_fusion')}")
    print(f"  Recommended mode: {get_recommended_mode()}")