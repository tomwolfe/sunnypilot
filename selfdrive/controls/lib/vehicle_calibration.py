#!/usr/bin/env python3
"""
Vehicle-specific calibration module for sunnypilot
Handles vehicle-specific parameter adjustments and validation
"""

import json
import os
from typing import Dict, Any, Optional
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from .autonomous_params import ADAPTIVE_BEHAVIOR_PARAMS, PERFORMANCE_PARAMS, ENVIRONMENTAL_PARAMS


class VehicleCalibration:
    """
    Handles vehicle-specific parameter calibration and validation
    """
    
    def __init__(self, car_fingerprint: Optional[str] = None):
        self.car_fingerprint = car_fingerprint
        self.params = Params()
        self.calibration_data = self._load_calibration_data()
        self.vehicle_specific_params = self._get_vehicle_specific_params()
        
    def _load_calibration_data(self) -> Dict[str, Any]:
        """Load calibration data from persistent storage or defaults"""
        try:
            calibration_file = self.params.get("VehicleCalibrationData", encoding='utf-8')
            if calibration_file:
                return json.loads(calibration_file)
            else:
                return self._get_default_calibration()
        except Exception as e:
            cloudlog.error(f"Error loading calibration data: {e}")
            return self._get_default_calibration()
    
    def _get_default_calibration(self) -> Dict[str, Any]:
        """Get default calibration values"""
        return {
            'vehicle_specific': {
                'lateral_accel_limit': 3.0,  # Base lateral acceleration limit
                'sharp_curve_threshold': 0.01,  # Adjusted per vehicle dynamics
                'thermal_threshold': 75.0,  # Default thermal threshold
                'min_follow_distance': 30.0,  # Base following distance
            },
            'calibration_status': {
                'completed': False,
                'last_calibration_date': None,
                'confidence_score': 0.5,  # Default confidence in calibration
            }
        }
    
    def _get_vehicle_specific_params(self) -> Dict[str, float]:
        """Get parameters that vary by vehicle type"""
        # Default values that may be overridden by vehicle-specific calibration
        base_params = {
            **ADAPTIVE_BEHAVIOR_PARAMS,
            **PERFORMANCE_PARAMS,
            **ENVIRONMENTAL_PARAMS
        }
        
        if self.car_fingerprint:
            # Apply vehicle-specific adjustments if known
            vehicle_params = self._get_vehicle_adjustments(self.car_fingerprint)
            base_params.update(vehicle_params)
        
        return base_params
    
    def _get_vehicle_adjustments(self, car_fingerprint: str) -> Dict[str, float]:
        """Get vehicle-specific parameter adjustments"""
        # Example vehicle-specific adjustments (these would be based on actual vehicle data)
        vehicle_adjustments = {
            # Example: Sports car might have higher lateral accel limits
            "SPORTS_CAR_EXAMPLE": {
                'BASE_LATERAL_ACCEL_LIMIT': 3.5,
                'SHARP_CURVE_THRESHOLD': 0.015,
                'THERMAL_THRESHOLD': 80.0,  # Higher thermal threshold for better cooling
            },
            # Example: SUV might have lower limits due to higher center of gravity
            "SUV_EXAMPLE": {
                'BASE_LATERAL_ACCEL_LIMIT': 2.5,
                'SHARP_CURVE_THRESHOLD': 0.008,
                'THERMAL_THRESHOLD': 70.0,  # Lower threshold for thermal protection
            }
        }
        
        return vehicle_adjustments.get(car_fingerprint, {})
    
    def validate_parameters(self) -> Dict[str, bool]:
        """
        Validate that all parameters are within safe ranges for this vehicle
        """
        validation_results = {}
        
        # Validate lateral acceleration limits
        lat_accel_limit = self.vehicle_specific_params.get('BASE_LATERAL_ACCEL_LIMIT', 3.0)
        validation_results['lateral_accel_limit'] = 1.0 <= lat_accel_limit <= 5.0
        
        # Validate sharp curve threshold
        curve_threshold = self.vehicle_specific_params.get('SHARP_CURVE_THRESHOLD', 0.01)
        validation_results['sharp_curve_threshold'] = 0.001 <= curve_threshold <= 0.05
        
        # Validate thermal threshold
        thermal_threshold = self.vehicle_specific_params.get('THERMAL_THRESHOLD', 75.0)
        validation_results['thermal_threshold'] = 60.0 <= thermal_threshold <= 90.0
        
        # Validate follow distance
        follow_distance = self.vehicle_specific_params.get('MIN_FOLLOW_DISTANCE', 30.0)
        validation_results['follow_distance'] = 15.0 <= follow_distance <= 100.0
        
        return validation_results
    
    def get_calibrated_params(self) -> Dict[str, Any]:
        """
        Get parameters adjusted for this specific vehicle
        """
        calibrated_params = self.vehicle_specific_params.copy()
        
        # Apply additional calibration factors if available
        if self.calibration_data.get('calibration_status', {}).get('completed', False):
            calibration_factors = self.calibration_data.get('factors', {})
            
            # Apply calibration factors to relevant parameters
            for param_name, factor in calibration_factors.items():
                if param_name in calibrated_params:
                    calibrated_params[param_name] *= factor
        
        return calibrated_params
    
    def update_calibration(self, calibration_data: Dict[str, Any]) -> bool:
        """
        Update calibration data and persist to storage
        """
        try:
            self.calibration_data.update(calibration_data)
            # Persist to params for next startup
            self.params.put("VehicleCalibrationData", json.dumps(self.calibration_data))
            cloudlog.info("Vehicle calibration updated successfully")
            return True
        except Exception as e:
            cloudlog.error(f"Error updating calibration: {e}")
            return False


def get_calibrated_params_for_vehicle(car_fingerprint: Optional[str] = None) -> Dict[str, Any]:
    """
    Convenience function to get properly calibrated parameters for a vehicle
    """
    calibrator = VehicleCalibration(car_fingerprint)
    return calibrator.get_calibrated_params()


def validate_vehicle_params(car_fingerprint: Optional[str] = None) -> Dict[str, bool]:
    """
    Convenience function to validate parameters for a vehicle
    """
    calibrator = VehicleCalibration(car_fingerprint)
    return calibrator.validate_parameters()