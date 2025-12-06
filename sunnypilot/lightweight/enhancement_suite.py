"""
Lightweight Enhancement Suite for Snapdragon 845
Main integration module combining all lightweight components
"""

from typing import Any
from sunnypilot.lightweight.scene_detection.detector import (
    create_scene_change_detector
)
from sunnypilot.lightweight.edge_case_detection.detector import (
    DetectionResult, create_edge_case_detector
)
from sunnypilot.lightweight.coordinated_control.controller import (
    create_coordinated_controller
)


class LightweightEnhancementSuite:
    """
    Main integration class for all lightweight enhancements optimized for Snapdragon 845.
    Combines scene detection, edge case detection, and coordinated control in a hardware-efficient way.
    """

    def __init__(self):
        # Initialize all lightweight components
        self.scene_detector = create_scene_change_detector()
        self.edge_case_detector = create_edge_case_detector()
        self.coordinated_controller, self.safety_limiter = create_coordinated_controller()

        # Performance tracking
        self.total_frames = 0
        self.frames_skipped = 0

    def should_process_frame(self, frame) -> tuple[bool, float]:
        """
        Determine if the current frame should be processed based on scene detection.

        Args:
            frame: Current camera frame

        Returns:
            Tuple of (should_process, motion_level)
        """
        self.total_frames += 1
        should_skip, motion_level = self.scene_detector.detect_change(frame)
        should_process = not should_skip

        if should_skip:
            self.frames_skipped += 1

        return should_process, motion_level

    def adjust_acceleration_for_lateral_demand(self,
                                             base_acceleration: float,
                                             lateral_demand: float,
                                             speed: float,
                                             curvature: float,
                                             radar_data: dict[str, Any],
                                             car_state: dict[str, Any]) -> float:
        """
        Adjust longitudinal acceleration based on lateral demand with safety limits.

        Args:
            base_acceleration: Original longitudinal acceleration command
            lateral_demand: Current lateral acceleration demand
            speed: Current vehicle speed
            curvature: Current path curvature
            radar_data: Radar information for safety checks
            car_state: Current car state for context

        Returns:
            Adjusted acceleration after coordinated control
        """
        # Apply coordinated control adjustment
        adjusted_accel = self.coordinated_controller.adjust_acceleration(
            base_acceleration, lateral_demand, speed, curvature
        )

        # Apply safety limits using radar data if available
        lead_distance = float('inf')
        lead_velocity = 0.0
        lat_accel = speed * speed * abs(curvature)

        if 'leadOne' in radar_data and radar_data['leadOne'].status:
            lead_distance = radar_data['leadOne'].dRel
            lead_velocity = speed - radar_data['leadOne'].vRel  # Lead's absolute velocity

        # Apply safety limits
        final_accel = self.safety_limiter.apply_safety_limits(
            adjusted_accel, base_acceleration, lat_accel, speed, lead_distance, lead_velocity
        )

        return final_accel

    def detect_edge_cases(self, radar_data: dict[str, Any],
                         vision_data: dict[str, Any],
                         car_state: dict[str, Any]) -> DetectionResult:
        """
        Detect critical edge cases that require special handling.

        Args:
            radar_data: Radar information
            vision_data: Vision model information
            car_state: Current car state

        Returns:
            DetectionResult with edge cases and recommended actions
        """
        return self.edge_case_detector.detect_edge_cases(radar_data, vision_data, car_state)

    def get_performance_stats(self) -> dict[str, Any]:
        """
        Get performance statistics for the enhancement suite.

        Returns:
            Dictionary with performance metrics
        """
        frame_skip_rate = self.frames_skipped / max(1, self.total_frames) if self.total_frames > 0 else 0.0

        return {
            'total_frames': self.total_frames,
            'frames_skipped': self.frames_skipped,
            'frame_skip_rate': frame_skip_rate,
            'efficiency_gain': (1.0 - frame_skip_rate) * 100,  # Percentage of frames that needed processing
            'scene_detection_active': True,
            'coordinated_control_active': True,
            'edge_case_detection_active': True
        }

    def reset_performance_stats(self):
        """Reset performance statistics."""
        self.total_frames = 0
        self.frames_skipped = 0


def create_light_enhancement_suite() -> LightweightEnhancementSuite:
    """
    Factory function to create the complete lightweight enhancement suite.
    """
    return LightweightEnhancementSuite()
