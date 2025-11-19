"""
Simple Traffic Sign Detection Handler for Sunnypilot
Minimal implementation to process model outputs for traffic sign information
"""
import numpy as np
from typing import List
import cereal.messaging as messaging
from cereal import log
from openpilot.common.swaglog import cloudlog
from sunnypilot.selfdrive.controls.lib.traffic_light_validation import TrafficSignType, TrafficSignData


class SimpleTrafficSignHandler:
    """
    Simple handler for processing traffic sign detection from model outputs
    """

    def __init__(self):
        # Simple mapping for traffic sign types
        self.traffic_sign_classes = {
            13: TrafficSignType.STOP_SIGN,
            14: TrafficSignType.TRAFFIC_LIGHT_RED,
            15: TrafficSignType.TRAFFIC_LIGHT_YELLOW,
            16: TrafficSignType.TRAFFIC_LIGHT_GREEN,
            18: TrafficSignType.SPEED_LIMIT,
        }

    def process_modeld_output(self, model_data: log.ModelDataV2) -> List[TrafficSignData]:
        """
        Process raw model data to extract traffic sign information
        """
        traffic_signs = []

        try:
            if hasattr(model_data, 'objects') and model_data.objects:
                for obj in model_data.objects:
                    if (hasattr(obj, 'confidence') and obj.confidence > 0.7 and
                        hasattr(obj, 'classType') and obj.classType in self.traffic_sign_classes):
                        
                        sign_type = self.traffic_sign_classes[obj.classType]
                        
                        # Calculate position
                        pos_x = getattr(obj, 'x', 0.0)
                        pos_y = getattr(obj, 'y', 0.0)
                        pos_z = getattr(obj, 'z', 0.0)
                        position = np.array([pos_x, pos_y, pos_z])
                        
                        # Calculate distance
                        distance = np.sqrt(pos_x**2 + pos_y**2) if hasattr(obj, 'x') and hasattr(obj, 'y') else 50.0
                        
                        sign_data = TrafficSignData(
                            sign_type=sign_type,
                            distance=distance,
                            confidence=obj.confidence,
                            position=position,
                            additional_data={}
                        )
                        traffic_signs.append(sign_data)

        except Exception as e:
            cloudlog.error(f"Error processing traffic signs from model data: {e}")

        return traffic_signs