"""
Lightweight Integration Module for Sunnypilot2 Enhancement Features

This module provides lightweight integration of the most valuable features
from the enhancement proposal without the computational overhead of the full system.
"""

from typing import Dict, Any, Tuple, Optional
import numpy as np

# Import our lightweight components
from sunnypilot.modeld.scene_change_detection import (
    LightweightSceneChangeDetector, 
    create_scene_change_detector
)
from sunnypilot.controls.lib.simplified_coordinated_control import (
    SimplifiedCoordinatedController, 
    SafetyLimiter,
    create_coordinated_controller
)
from sunnypilot.controls.lib.basic_edge_case_detection import (
    BasicEdgeCaseDetector,
    DetectionResult,
    create_edge_case_detector
)


class LightEnhancementIntegrator:
    """
    Lightweight integrator for the most valuable enhancement features.
    
    This class integrates scene change detection, coordinated control, and 
    basic edge case detection while maintaining hardware compatibility.
    """
    
    def __init__(self):
        """Initialize the lightweight enhancement integrator."""
        # Initialize the lightweight components
        self.scene_detector = create_scene_change_detector()
        self.coordinated_controller, self.safety_limiter = create_coordinated_controller()
        self.edge_case_detector = create_edge_case_detector()
        
        # Performance monitoring
        self.processing_skipped_count = 0
        self.total_frames = 0
        
    def should_process_frame(self, current_frame: np.ndarray) -> Tuple[bool, float]:
        """
        Determine if the current frame should be processed based on scene change detection.
        
        Args:
            current_frame: Current camera frame to analyze
            
        Returns:
            Tuple of (should_process, motion_level)
        """
        self.total_frames += 1
        skip_processing, motion_level = self.scene_detector.detect_change(current_frame)
        
        if skip_processing:
            self.processing_skipped_count += 1
            
        return not skip_processing, motion_level
    
    def adjust_acceleration_for_lateral_demand(self,
                                             base_acceleration: float,
                                             lateral_demand: float,
                                             speed: float,
                                             curvature: float,
                                             radar_data: Dict,
                                             car_state: Dict) -> float:
        """
        Adjust acceleration based on lateral demand and safety considerations.
        
        Args:
            base_acceleration: Base acceleration from planner
            lateral_demand: Current lateral demand
            speed: Current speed
            curvature: Current path curvature
            radar_data: Radar data for safety checks
            car_state: Car state data for safety checks
            
        Returns:
            Adjusted acceleration with coordination and safety limits
        """
        # Apply coordinated control
        adjusted_accel = self.coordinated_controller.adjust_acceleration(
            base_acceleration, lateral_demand, speed, curvature
        )
        
        # Apply safety limits
        # Extract relevant data for safety checks
        lead_distance = float('inf')
        lead_velocity = 0.0
        
        if 'leadOne' in radar_data and radar_data['leadOne'].status:
            lead = radar_data['leadOne']
            lead_distance = lead.dRel
            lead_velocity = lead.vRel + car_state.get('vEgo', 0.0)  # Absolute velocity
        
        lat_accel = speed * speed * abs(curvature) if curvature else 0.0
        
        final_accel = self.safety_limiter.apply_safety_limits(
            adjusted_accel, base_acceleration, lat_accel, 
            speed, lead_distance, lead_velocity
        )
        
        return final_accel
    
    def detect_edge_cases(self, 
                         radar_data: Dict,
                         vision_data: Dict,
                         car_state: Dict) -> DetectionResult:
        """
        Detect critical edge cases using lightweight detection.
        
        Args:
            radar_data: Radar data from the car
            vision_data: Vision data from cameras
            car_state: Current car state
            
        Returns:
            Detection result with edge cases and recommendations
        """
        return self.edge_case_detector.detect_edge_cases(radar_data, vision_data, car_state)
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """
        Get performance statistics for the enhancement system.
        
        Returns:
            Dictionary with performance statistics
        """
        skip_rate = (self.processing_skipped_count / self.total_frames) if self.total_frames > 0 else 0
        
        return {
            'frame_skip_rate': skip_rate,
            'frames_processed': self.total_frames - self.processing_skipped_count,
            'frames_skipped': self.processing_skipped_count,
            'total_frames': self.total_frames
        }
    
    def reset(self):
        """Reset all component states."""
        self.scene_detector.reset()
        self.processing_skipped_count = 0
        self.total_frames = 0


def create_light_enhancement_integrator() -> LightEnhancementIntegrator:
    """
    Factory function to create a lightweight enhancement integrator.
    """
    return LightEnhancementIntegrator()


# Example usage function showing how to integrate with existing system
def integrate_with_control_loop(
    current_frame: np.ndarray,
    base_acceleration: float,
    lateral_demand: float,
    speed: float,
    curvature: float,
    radar_data: Dict,
    vision_data: Dict,
    car_state: Dict
) -> Tuple[bool, float, DetectionResult]:
    """
    Example integration with an existing control loop.
    
    Args:
        current_frame: Current camera frame
        base_acceleration: Base acceleration from planner
        lateral_demand: Current lateral demand
        speed: Current speed
        curvature: Current path curvature
        radar_data: Radar information
        vision_data: Vision information
        car_state: Current car state
        
    Returns:
        Tuple of (should_process_frame, final_acceleration, edge_case_result)
    """
    # This would typically be a singleton instance shared across the system
    integrator = create_light_enhancement_integrator()
    
    # Check if we should process this frame
    should_process, motion_level = integrator.should_process_frame(current_frame)
    
    final_acceleration = base_acceleration
    edge_case_result = DetectionResult([], 1.0, "CONTINUE_NORMAL")
    
    if should_process:
        # Apply coordinated control
        final_acceleration = integrator.adjust_acceleration_for_lateral_demand(
            base_acceleration, lateral_demand, speed, curvature, radar_data, car_state
        )
        
        # Detect edge cases
        edge_case_result = integrator.detect_edge_cases(radar_data, vision_data, car_state)
        
        # Apply speed multiplier from edge case detection if needed
        final_acceleration *= edge_case_result.safe_speed_multiplier
    
    return should_process, final_acceleration, edge_case_result