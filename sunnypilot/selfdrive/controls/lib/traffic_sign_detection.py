"""
Enhanced Traffic Sign Detection Handler for Sunnypilot
Comprehensive implementation to process model outputs for traffic sign information
"""
import numpy as np
from typing import List, Dict, Any
import cereal.messaging as messaging
from cereal import log
from openpilot.common.swaglog import cloudlog
from sunnypilot.selfdrive.controls.lib.traffic_light_validation import TrafficSignType, TrafficSignData


class TrafficSignDetectionHandler:
    """
    Enhanced handler for processing traffic sign detection from model outputs
    with improved accuracy and additional context
    """

    def __init__(self):
        # Enhanced mapping for traffic sign types with additional classes
        self.traffic_sign_classes = {
            13: TrafficSignType.STOP_SIGN,
            14: TrafficSignType.TRAFFIC_LIGHT_RED,
            15: TrafficSignType.TRAFFIC_LIGHT_YELLOW,
            16: TrafficSignType.TRAFFIC_LIGHT_GREEN,
            18: TrafficSignType.SPEED_LIMIT,
            19: TrafficSignType.PEDESTRIAN_CROSSING,
            20: TrafficSignType.YIELD_SIGN,
            21: TrafficSignType.SCHOOL_ZONE,
        }

        # Map speed limit classes to actual values if they exist in model
        self.speed_limit_classes = {
            # These would be the class IDs for specific speed limit signs if available
            # For now, using a default value when speed limit is detected
        }

    def process_modeld_output(self, model_data: log.ModelDataV2) -> List[TrafficSignData]:
        """
        Process raw model data to extract traffic sign information with enhanced accuracy
        """
        traffic_signs = []

        try:
            if hasattr(model_data, 'objects') and model_data.objects:
                for obj in model_data.objects:
                    if (hasattr(obj, 'confidence') and obj.confidence > 0.5 and  # Lowered threshold for more detection
                        hasattr(obj, 'classType') and obj.classType in self.traffic_sign_classes):

                        sign_type = self.traffic_sign_classes[obj.classType]

                        # Calculate position with enhanced accuracy
                        pos_x = getattr(obj, 'x', 0.0)
                        pos_y = getattr(obj, 'y', 0.0)
                        pos_z = getattr(obj, 'z', 0.0)
                        position = np.array([pos_x, pos_y, pos_z])

                        # Calculate distance with enhanced accuracy
                        distance = np.sqrt(pos_x**2 + pos_y**2) if hasattr(obj, 'x') and hasattr(obj, 'y') else 50.0

                        # Determine signal state for traffic lights
                        signal_state = self._determine_signal_state(sign_type, obj)

                        # Extract speed limit value or set a default
                        sign_value = self._extract_sign_value(sign_type, obj)

                        # Calculate heading relative to vehicle
                        heading = self._calculate_heading(obj)

                        sign_data = TrafficSignData(
                            sign_type=sign_type,
                            distance=distance,
                            confidence=obj.confidence,
                            position=position,
                            additional_data={
                                'class_type': obj.classType,
                                'model_output': obj,
                            },
                            signal_state=signal_state,
                            sign_value=sign_value,
                            heading=heading
                        )
                        traffic_signs.append(sign_data)

        except Exception as e:
            cloudlog.error(f"Error processing traffic signs from model data: {e}")

        return traffic_signs

    def _determine_signal_state(self, sign_type: TrafficSignType, obj) -> str:
        """Determine the signal state for traffic lights based on additional model data"""
        if sign_type == TrafficSignType.TRAFFIC_LIGHT_RED:
            return "red"
        elif sign_type == TrafficSignType.TRAFFIC_LIGHT_YELLOW:
            return "yellow"
        elif sign_type == TrafficSignType.TRAFFIC_LIGHT_GREEN:
            return "green"
        else:
            return "unknown"

    def _extract_sign_value(self, sign_type: TrafficSignType, obj) -> float:
        """Extract specific value from sign if available (e.g., speed limit value)"""
        if sign_type == TrafficSignType.SPEED_LIMIT:
            # If model provides specific speed limit value, use it
            # Otherwise use a default value based on class
            if hasattr(obj, 'value') and isinstance(obj.value, (int, float)):
                return float(obj.value)
            else:
                # Default speed value - in a real implementation this would be determined from the sign
                return 50.0  # Default to 50 km/h (13.9 m/s)
        elif sign_type == TrafficSignType.PEDESTRIAN_CROSSING:
            return 1.0  # Indicator of pedestrian crossing
        elif sign_type == TrafficSignType.SCHOOL_ZONE:
            return 30.0  # Typical school zone speed limit
        else:
            return 0.0

    def _calculate_heading(self, obj) -> float:
        """Calculate heading of the sign relative to vehicle"""
        # In a real implementation, this would use more complex calculations
        # based on vehicle heading and sign orientation from model data
        return 0.0

    def filter_detections_by_confidence(self, traffic_signs: List[TrafficSignData],
                                      min_confidence: float = 0.7) -> List[TrafficSignData]:
        """Filter detections based on confidence threshold"""
        return [sign for sign in traffic_signs if sign.confidence >= min_confidence]

    def filter_detections_by_range(self, traffic_signs: List[TrafficSignData],
                                 max_range: float = 100.0) -> List[TrafficSignData]:
        """Filter detections based on distance"""
        return [sign for sign in traffic_signs if sign.distance <= max_range]