"""
Traffic Sign Detection Handler for Sunnypilot
Processes model outputs to extract traffic sign information for validation
"""
import numpy as np
from typing import Dict, Any, List, Optional
import cereal.messaging as messaging
from cereal import log
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.traffic_light_validation import TrafficSignType, TrafficSignData


class TrafficSignDetectionHandler:
    """
    Handler for processing traffic sign detection from model outputs
    """
    
    def __init__(self):
        self.traffic_sign_types = {
            0: TrafficSignType.STOP_SIGN,
            1: TrafficSignType.YIELD_SIGN, 
            2: TrafficSignType.TRAFFIC_LIGHT_RED,
            3: TrafficSignType.TRAFFIC_LIGHT_YELLOW,
            4: TrafficSignType.TRAFFIC_LIGHT_GREEN,
            5: TrafficSignType.SPEED_LIMIT,
            6: TrafficSignType.PEDESTRIAN_CROSSING,
            7: TrafficSignType.SCHOOL_ZONE,
            8: TrafficSignType.CONSTRUCTION_ZONE
        }
        
        # Default confidence threshold for traffic sign detection
        self.confidence_threshold = 0.7
        
    def process_modeld_output(self, model_data: log.ModelDataV2) -> List[TrafficSignData]:
        """
        Process raw model data to extract traffic sign information
        """
        traffic_signs = []
        
        try:
            # Process detected objects from the model
            if hasattr(model_data, 'objects') and model_data.objects:
                for obj in model_data.objects:
                    # Check if object is a traffic sign based on class and confidence
                    if obj.confidence > self.confidence_threshold:
                        sign_type = self._map_object_to_sign_type(obj)
                        if sign_type:
                            # Calculate distance and position from ego vehicle
                            distance = self._calculate_distance_from_model(obj)
                            position = np.array([obj.x, obj.y, obj.z]) if hasattr(obj, 'x') and hasattr(obj, 'y') and hasattr(obj, 'z') else np.array([0.0, 0.0, 0.0])
                            
                            sign_data = TrafficSignData(
                                sign_type=sign_type,
                                distance=distance,
                                confidence=obj.confidence,
                                position=position,
                                validity_time=model_data.frameId / 20.0 + 5.0  # Assume 20Hz and 5s validity
                            )
                            traffic_signs.append(sign_data)
                            
        except Exception as e:
            cloudlog.error(f"Error processing traffic signs from model data: {e}")
            
        return traffic_signs
    
    def process_custom_model_output(self, custom_model_data: Any) -> List[TrafficSignData]:
        """
        Process custom model data that may have explicit traffic sign information
        """
        traffic_signs = []
        
        try:
            # If the model has explicit traffic sign detection
            if hasattr(custom_model_data, 'trafficSigns') and custom_model_data.trafficSigns:
                for sign in custom_model_data.trafficSigns:
                    sign_type = self._map_sign_id_to_type(getattr(sign, 'type', -1))
                    if sign_type:
                        sign_data = TrafficSignData(
                            sign_type=sign_type,
                            distance=getattr(sign, 'distance', 0.0),
                            confidence=getattr(sign, 'confidence', 0.5),
                            position=np.array([getattr(sign, 'x', 0.0), 
                                             getattr(sign, 'y', 0.0), 
                                             getattr(sign, 'z', 0.0)]),
                            validity_time=getattr(sign, 'validityTime', 0.0),
                            additional_data={
                                'size': getattr(sign, 'size', None),
                                'color': getattr(sign, 'color', None),
                                'value': getattr(sign, 'value', None)  # for speed limit signs
                            }
                        )
                        traffic_signs.append(sign_data)
                        
        except Exception as e:
            cloudlog.error(f"Error processing custom traffic sign data: {e}")
            
        return traffic_signs
    
    def _map_object_to_sign_type(self, obj) -> Optional[TrafficSignType]:
        """
        Map detected object to traffic sign type based on model class
        """
        # This mapping will depend on how the model classifies objects
        # Common class indices for traffic signs (these are examples)
        class_map = {
            13: TrafficSignType.STOP_SIGN,        # Common class ID for stop signs
            14: TrafficSignType.TRAFFIC_LIGHT_RED,  # Traffic light red
            15: TrafficSignType.TRAFFIC_LIGHT_YELLOW,
            16: TrafficSignType.TRAFFIC_LIGHT_GREEN,
            17: TrafficSignType.YIELD_SIGN,
            18: TrafficSignType.SPEED_LIMIT,
            19: TrafficSignType.PEDESTRIAN_CROSSING,
        }
        
        # Get the classification with highest confidence that meets threshold
        if hasattr(obj, 'classType') and obj.classType in class_map:
            return class_map[obj.classType]
            
        return None
    
    def _map_sign_id_to_type(self, sign_id: int) -> Optional[TrafficSignType]:
        """
        Map sign ID to traffic sign type
        """
        if sign_id in self.traffic_sign_types:
            return self.traffic_sign_types[sign_id]
        return None
    
    def _calculate_distance_from_model(self, obj) -> float:
        """
        Calculate distance from ego vehicle based on model output
        """
        # If model provides distance directly
        if hasattr(obj, 'distance'):
            return obj.distance
        # If object has x,y coordinates relative to ego
        elif hasattr(obj, 'x') and hasattr(obj, 'y'):
            return np.sqrt(obj.x**2 + obj.y**2)
        else:
            return 50.0  # Default distance if not available

    def integrate_with_traffic_validator(self,
                                       traffic_validator: Any,
                                       model_data: log.ModelDataV2) -> None:
        """
        Integrate detected traffic signs with the traffic validation system
        """
        # Process model data to get traffic signs
        detected_signs = self.process_modeld_output(model_data)

        # Add each detected sign to the validator
        for sign in detected_signs:
            traffic_validator.add_traffic_sign_data(sign)


def create_traffic_sign_handler() -> TrafficSignDetectionHandler:
    """Factory function to create traffic sign detection handler"""
    return TrafficSignDetectionHandler()