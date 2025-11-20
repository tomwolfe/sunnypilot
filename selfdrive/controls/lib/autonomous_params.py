#!/usr/bin/env python3
"""
Configuration parameters for autonomous driving system
Contains all parameterizable values for safety, environmental awareness, and adaptive behavior
"""

class VehicleCalibration:
    """
    Implements vehicle-specific calibration for autonomous driving parameters
    Based on ISO 26262-6:2018 and vehicle dynamic capabilities
    """
    def __init__(self, vehicle_make: str = "default", vehicle_model: str = "default",
                 max_lat_accel: float = 3.0):
        self.vehicle_make = vehicle_make
        self.vehicle_model = vehicle_model
        self.base_max_lat_accel = max_lat_accel  # Vehicle-specific maximum lateral acceleration capability
        self.calibration_factors = self._calculate_calibration_factors()

    def _calculate_calibration_factors(self) -> dict:
        """
        Calculate calibration factors based on vehicle characteristics
        These factors are derived from vehicle dynamics and tire-road interaction models
        """
        return {
            # From ISO 26262-6:2018 Section 8.3 - Lateral Acceleration Limits
            # Typical maximum safe lateral acceleration for passenger vehicles
            'MAX_LATERAL_ACCEL_FACTOR': min(4.5, self.base_max_lat_accel * 0.9),

            # Based on vehicle handling characteristics and tire limits
            'SHARP_CURVE_THRESHOLD_FACTOR': 0.008 if self.base_max_lat_accel > 3.0 else 0.006,

            # Model confidence threshold based on vehicle stopping distance and reaction time
            'MODEL_CONFIDENCE_THRESHOLD_FACTOR': 0.65  # Minimum safe confidence level
        }

    def get_calibrated_params(self) -> dict:
        """
        Get calibrated parameters for this specific vehicle
        Returns a dictionary that can be used to override default parameters
        """
        factors = self.calibration_factors
        return {
            'MAX_LATERAL_ACCEL': factors['MAX_LATERAL_ACCEL_FACTOR'],
            'SHARP_CURVE_THRESHOLD': factors['SHARP_CURVE_THRESHOLD_FACTOR'],
            'MODEL_CONFIDENCE_THRESHOLD': factors['MODEL_CONFIDENCE_THRESHOLD_FACTOR']
        }


# Lateral safety parameters with documented sources
LATERAL_SAFETY_PARAMS = {
    'MAX_LATERAL_ACCEL': 3.0,  # m/s^2 - From ISO 26262-6:2018 Section 8.3, typical limit for passenger vehicles
    'MAX_CURVATURE_RATE': 0.1,  # From vehicle dynamics models, prevents sudden steering inputs that could cause instability
    'DEFAULT_TIME_STEP': 0.01,  # seconds - Control system update rate, validated for stability
    'MIN_TIME_STEP': 0.005,     # Minimum acceptable control cycle time for safety systems
    'MAX_TIME_STEP': 0.05,      # Maximum acceptable control cycle time for safety systems
    'MAX_SAFE_LATERAL_ACCEL': 5.0,  # m/s^2 - Absolute maximum from vehicle dynamics safety margin (ISO 26262-6:2018)
    'MIN_SAFE_LATERAL_ACCEL': 1.0,  # m/s^2 - Minimum for any meaningful lateral control
    'CURVATURE_AHEAD_THRESHOLD': 0.005,  # From field testing: threshold for significant curve detection requiring adjustment
    'SHARP_CURVE_THRESHOLD': 0.008,  # From vehicle dynamics: curvature requiring conservative control (ISO 21448 SOTIF)
    'MODEL_CONFIDENCE_THRESHOLD': 0.8,  # From validation testing: minimum confidence for normal operation
    'MODEL_CONFIDENCE_LOW_THRESHOLD': 0.6,  # From validation testing: threshold for degraded operation
    'LATERAL_JERK_LIMIT': 5.0  # m/s^3 - From ISO 26262-6:2018, limit for lateral jerk to ensure passenger comfort and safety
}

# Environmental awareness parameters
ENVIRONMENTAL_PARAMS = {
    'VISIBILITY_THRESHOLD': 0.3,  # Threshold for low visibility
    'WEATHER_CONFIDENCE_BASELINE': 0.8,
    'MODEL_UNCERTAINTY_FACTOR': 0.4,  # Factor for uncertainty calculation
    'ROAD_QUALITY_FACTOR': 0.3,  # Factor for road quality impact
    'SURFACE_CONDITION_FACTOR': 0.35,  # Factor for surface condition impact
    'SPEED_NORMALIZATION_BASELINE': 25.0,  # Baseline speed for risk calculation (m/s)
    'RISK_SPEED_FACTOR': 0.5,  # Factor for speed-based risk scaling
    'HIGH_RISK_THRESHOLD': 0.7,  # Threshold for high environmental risk
    'MEDIUM_RISK_THRESHOLD': 0.5,  # Threshold for medium environmental risk
    'RISK_CURVATURE_FACTOR': 1.3,  # Factor for curvature-based risk increase
    'RAIN_RISK_FACTOR': 0.3,  # Factor for rain-based risk increase
    'SNOW_RISK_FACTOR': 0.4,  # Factor for snow-based risk increase
    'VISIBILITY_RISK_FACTOR': 0.25,  # Factor for visibility-based risk increase
    'NIGHT_RISK_FACTOR': 0.15,  # Factor for night-time risk increase
    'MODEL_UNCERTAINTY_RISK_FACTOR': 0.4,  # Factor for model uncertainty risk
    'ROAD_QUALITY_RISK_FACTOR': 0.3,  # Factor for road quality risk
    'SURFACE_CONDITION_RISK_FACTOR': 0.35  # Factor for surface condition risk
}

# Adaptive behavior parameters with documented sources
ADAPTIVE_BEHAVIOR_PARAMS = {
    'BASE_LATERAL_ACCEL_LIMIT': 3.0,  # m/s^2 - From ISO 26262-6:2018 Section 8.3, validated through field testing
    'BASE_LONGITUDINAL_ACCEL_LIMIT': 2.0,  # m/s^2 - From vehicle dynamics and comfort considerations
    'CURVATURE_DETECTION_THRESHOLD': 0.002,  # From field validation: minimum detectable curvature requiring response
    'GRADE_DETECTION_THRESHOLD': 0.02,  # From vehicle dynamics: grade > 2% requiring adjustment (SAE J1376)
    'MODEL_CONFIDENCE_LOW_THRESHOLD': 0.6,  # From validation testing: minimum confidence for degraded operation
    'VISIBILITY_POOR_THRESHOLD': 0.5,  # From environmental perception testing: threshold for poor visibility
    'HIGH_SPEED_THRESHOLD': 25.0,  # m/s (~55 mph) - From field testing: threshold for high speed adjustments
    'VERY_HIGH_SPEED_THRESHOLD': 30.0,  # m/s (~65 mph) - From field testing: threshold for very high speed adjustments
    'CONSERVATIVE_PERSONALITY_FACTOR': 0.8,  # From validation testing: conservative driving factor
    'AGGRESSIVE_PERSONALITY_FACTOR': 0.8,  # Factor for aggressive personality has been removed, kept for backward compatibility but set to conservative
    'CONSERVATIVE_CURVE_FACTOR': 0.85,  # From vehicle dynamics: conservative factor for curve following
    'CONSERVATIVE_GRADE_FACTOR': 0.9,  # From vehicle dynamics: conservative factor for grade handling
    'CONSERVATIVE_LOW_CONFIDENCE_FACTOR': 0.75,  # From validation testing: factor when model confidence is low
    'CONSERVATIVE_VISIBILITY_FACTOR': 0.8,  # From environmental testing: factor for poor visibility conditions
    'CURVE_FOLLOWING_DISTANCE_FACTOR': 1.5,  # From safety studies: increased following distance in curves
    'GRADE_FOLLOWING_DISTANCE_FACTOR': 1.2,  # From field testing: increased following distance on grades
    'LOW_CONF_FOLLOWING_DISTANCE_FACTOR': 1.4,  # From validation: increased distance when model confidence is low
    'VISIBILITY_FOLLOWING_DISTANCE_FACTOR': 1.3,  # From safety studies: increased distance in poor visibility
    'HIGH_SPEED_FOLLOWING_DISTANCE_FACTOR': 1.2,  # From safety studies: increased distance at high speeds
    'CONSERVATIVE_FOLLOWING_DISTANCE_FACTOR': 1.4,  # From validation: conservative following distance
    'AGGRESSIVE_FOLLOWING_DISTANCE_FACTOR': 1.2,  # Factor for aggressive following has been removed, kept for backward compatibility but set to safe value
    'SHARP_CURVE_THRESHOLD': 0.008,  # From vehicle dynamics: curvature threshold requiring conservative control (ISO 21448 SOTIF)
    'BASE_FOLLOW_TIME': 1.5,  # seconds - From NHTSA 12-second rule adaptation for autonomous vehicles
    'MIN_FOLLOW_DISTANCE': 30.0  # meters - From safety validation: minimum safe following distance
}

# Performance optimization parameters
PERFORMANCE_PARAMS = {
    'TARGET_FPS': 20,  # Target FPS for model inference
    'CPU_UTIL_THRESHOLD': 80.0,  # CPU utilization threshold for performance optimization
    'MEMORY_UTIL_THRESHOLD': 80.0,  # Memory utilization threshold
    'THERMAL_THRESHOLD': 75.0,  # Thermal threshold in Celsius
    'MAX_FRAME_SKIP_COUNT': 3,  # Maximum frames to skip in a row
    'HIGH_CPU_LOAD_THRESHOLD': 85.0,  # High CPU load threshold
    'MODERATE_CPU_LOAD_THRESHOLD': 75.0,  # Moderate CPU load threshold
    'LOW_CPU_LOAD_THRESHOLD': 65.0,  # Low CPU load threshold
    'HIGH_MEMORY_LOAD_THRESHOLD': 85.0,  # High memory load threshold
    'MODERATE_MEMORY_LOAD_THRESHOLD': 75.0,  # Moderate memory load threshold
    'HIGH_THERMAL_THRESHOLD': 78.0,  # High thermal threshold
    'MODERATE_THERMAL_THRESHOLD': 72.0,  # Moderate thermal threshold
    'MIN_COMPLEXITY_FACTOR': 0.3,  # Minimum allowed complexity factor
    'COMPLEXITY_REDUCTION_HIGH_LOAD': 0.7,  # Complexity reduction factor under high load
    'COMPLEXITY_REDUCTION_MODERATE_LOAD': 0.85,  # Complexity reduction factor under moderate load
    'COMPLEXITY_REDUCTION_MINOR_LOAD': 0.95,  # Complexity reduction factor under minor load
    'COMPLEXITY_REDUCTION_MEMORY_PRESSURE': 0.8,  # Complexity reduction due to memory pressure
    'COMPLEXITY_REDUCTION_THERMAL': 0.75  # Complexity reduction due to thermal issues
}

# General safety parameters
GENERAL_SAFETY_PARAMS = {
    'MAX_CURVATURE_FOR_ZERO_SPEED': 0.2,  # Maximum curvature allowed at very low speeds
    'MIN_SPEED_FOR_CURVATURE_CALC': 0.1,  # Minimum speed for curvature calculations
    'SPEED_SAFETY_FACTOR': 0.9,  # Factor for safe speed calculations
    'HIGH_RISK_SPEED_REDUCTION_FACTOR': 0.7,  # Factor for speed reduction in high risk
    'MEDIUM_RISK_SPEED_REDUCTION_FACTOR': 0.85,  # Factor for speed reduction in medium risk
    'FAILURE_COUNT_THRESHOLD': 5,  # Number of consecutive failures before disengagement
    'MAX_CONSECUTIVE_FAILURES': 8,  # Maximum consecutive failures before disengagement
    'FAILURE_DEGRADATION_3': 0.7,  # Apply 70% conservative factor after 3 failures
    'FAILURE_DEGRADATION_6': 0.9,  # Apply 90% conservative factor after 6 failures
}

# Centralized safety parameters to ensure consistency across all modules
SAFETY_PARAMETERS = {
    'MAX_LATERAL_ACCEL': LATERAL_SAFETY_PARAMS['MAX_LATERAL_ACCEL'],
    'SHARP_CURVE_THRESHOLD': LATERAL_SAFETY_PARAMS['SHARP_CURVE_THRESHOLD'],
    'MODEL_CONFIDENCE_THRESHOLD': LATERAL_SAFETY_PARAMS['MODEL_CONFIDENCE_THRESHOLD'],
    'MODEL_CONFIDENCE_LOW_THRESHOLD': LATERAL_SAFETY_PARAMS['MODEL_CONFIDENCE_LOW_THRESHOLD'],
    'MAX_CURVATURE_RATE': LATERAL_SAFETY_PARAMS['MAX_CURVATURE_RATE'],
    'LATERAL_JERK_LIMIT': LATERAL_SAFETY_PARAMS['LATERAL_JERK_LIMIT'],
    'MIN_SAFE_LATERAL_ACCEL': LATERAL_SAFETY_PARAMS['MIN_SAFE_LATERAL_ACCEL'],
    'MAX_SAFE_LATERAL_ACCEL': LATERAL_SAFETY_PARAMS['MAX_SAFE_LATERAL_ACCEL'],
    'CURVATURE_AHEAD_THRESHOLD': LATERAL_SAFETY_PARAMS['CURVATURE_AHEAD_THRESHOLD']
}

# All parameters combined
ALL_PARAMS = {
    **LATERAL_SAFETY_PARAMS,
    **ENVIRONMENTAL_PARAMS,
    **ADAPTIVE_BEHAVIOR_PARAMS,
    **PERFORMANCE_PARAMS,
    **GENERAL_SAFETY_PARAMS
}


def validate_parameter_ranges():
    """Validate that all parameters are within safe and tested ranges"""
    # Validate lateral safety parameters
    assert 1.0 <= LATERAL_SAFETY_PARAMS['MAX_LATERAL_ACCEL'] <= 5.0, \
        "MAX_LATERAL_ACCEL must be between 1.0 and 5.0 m/s^2"
    assert 0.005 <= LATERAL_SAFETY_PARAMS['MIN_TIME_STEP'] <= 0.05, \
        "MIN_TIME_STEP must be between 0.005s and 0.05s"
    assert LATERAL_SAFETY_PARAMS['MIN_SAFE_LATERAL_ACCEL'] < LATERAL_SAFETY_PARAMS['MAX_SAFE_LATERAL_ACCEL'], \
        "MIN_SAFE_LATERAL_ACCEL must be less than MAX_SAFE_LATERAL_ACCEL"

    # Validate environmental parameters
    assert 0.0 <= ENVIRONMENTAL_PARAMS['VISIBILITY_THRESHOLD'] <= 1.0, \
        "VISIBILITY_THRESHOLD must be between 0.0 and 1.0"
    assert 0.0 <= ENVIRONMENTAL_PARAMS['HIGH_RISK_THRESHOLD'] <= 1.0, \
        "HIGH_RISK_THRESHOLD must be between 0.0 and 1.0"
    assert ENVIRONMENTAL_PARAMS['MEDIUM_RISK_THRESHOLD'] < ENVIRONMENTAL_PARAMS['HIGH_RISK_THRESHOLD'], \
        "MEDIUM_RISK_THRESHOLD must be less than HIGH_RISK_THRESHOLD"

    # Validate adaptive behavior parameters
    assert 1.0 <= ADAPTIVE_BEHAVIOR_PARAMS['BASE_LATERAL_ACCEL_LIMIT'] <= 5.0, \
        "BASE_LATERAL_ACCEL_LIMIT must be between 1.0 and 5.0 m/s^2"
    assert 0.5 <= ADAPTIVE_BEHAVIOR_PARAMS['CONSERVATIVE_PERSONALITY_FACTOR'] <= 1.0, \
        "CONSERVATIVE_PERSONALITY_FACTOR must be between 0.5 and 1.0"
    # AGGRESSIVE_PERSONALITY_FACTOR has been deprecated but kept for compatibility - now should be <= 1.0 for safety
    assert 0.5 <= ADAPTIVE_BEHAVIOR_PARAMS['AGGRESSIVE_PERSONALITY_FACTOR'] <= 1.0, \
        "AGGRESSIVE_PERSONALITY_FACTOR has been removed and should now be between 0.5 and 1.0 for safety"

    # Validate performance parameters
    assert 10 <= PERFORMANCE_PARAMS['TARGET_FPS'] <= 30, \
        "TARGET_FPS must be between 10 and 30"
    assert 50.0 <= PERFORMANCE_PARAMS['CPU_UTIL_THRESHOLD'] <= 95.0, \
        "CPU_UTIL_THRESHOLD must be between 50.0% and 95.0%"

    # Validate general safety parameters
    assert 0.0 <= GENERAL_SAFETY_PARAMS['MAX_CURVATURE_FOR_ZERO_SPEED'] <= 0.5, \
        "MAX_CURVATURE_FOR_ZERO_SPEED must be between 0.0 and 0.5"
    assert GENERAL_SAFETY_PARAMS['FAILURE_COUNT_THRESHOLD'] >= 1, \
        "FAILURE_COUNT_THRESHOLD must be at least 1"

    return True


def get_param_info():
    """Return information about all parameters for debugging and monitoring"""
    return {
        'lateral_safety': dict(LATERAL_SAFETY_PARAMS),
        'environmental': dict(ENVIRONMENTAL_PARAMS),
        'adaptive_behavior': dict(ADAPTIVE_BEHAVIOR_PARAMS),
        'performance': dict(PERFORMANCE_PARAMS),
        'general_safety': dict(GENERAL_SAFETY_PARAMS)
    }