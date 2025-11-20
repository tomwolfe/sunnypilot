#!/usr/bin/env python3
"""
Configuration parameters for autonomous driving system
Contains all parameterizable values for safety, environmental awareness, and adaptive behavior
"""

# Lateral safety parameters
LATERAL_SAFETY_PARAMS = {
    'MAX_LATERAL_ACCEL': 3.0,  # m/s^2 - Maximum lateral acceleration allowed (validated for safety)
    'MAX_CURVATURE_RATE': 0.1,  # Maximum rate of curvature change - prevents sudden steering inputs
    'DEFAULT_TIME_STEP': 0.01,  # seconds - Default time step for rate calculations (validated range: 0.005-0.05)
    'MIN_TIME_STEP': 0.005,     # Minimum acceptable time step (5ms)
    'MAX_TIME_STEP': 0.05,      # Maximum acceptable time step (50ms)
    'MAX_SAFE_LATERAL_ACCEL': 5.0,  # m/s^2 - absolute maximum for safety
    'MIN_SAFE_LATERAL_ACCEL': 1.0,  # m/s^2 - minimum for any meaningful control
    'CURVATURE_AHEAD_THRESHOLD': 0.005,  # Threshold for significant curve detection
    'SHARP_CURVE_THRESHOLD': 0.008,  # Threshold for sharp curve detection
    'MODEL_CONFIDENCE_THRESHOLD': 0.8,  # Threshold for low confidence detection
    'MODEL_CONFIDENCE_LOW_THRESHOLD': 0.6,  # Threshold for very low confidence detection
    'LATERAL_JERK_LIMIT': 5.0  # m/s^3 - limit for lateral jerk
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

# Adaptive behavior parameters
ADAPTIVE_BEHAVIOR_PARAMS = {
    'BASE_LATERAL_ACCEL_LIMIT': 3.0,  # Base lateral acceleration limit (m/s^2)
    'BASE_LONGITUDINAL_ACCEL_LIMIT': 2.0,  # Base longitudinal acceleration limit (m/s^2)
    'CURVATURE_DETECTION_THRESHOLD': 0.002,  # Threshold for curve detection
    'GRADE_DETECTION_THRESHOLD': 0.02,  # Threshold for grade detection (> 2%)
    'MODEL_CONFIDENCE_LOW_THRESHOLD': 0.6,  # Threshold for low model confidence
    'VISIBILITY_POOR_THRESHOLD': 0.5,  # Threshold for poor visibility
    'HIGH_SPEED_THRESHOLD': 25.0,  # Threshold for high speed (~55 mph)
    'VERY_HIGH_SPEED_THRESHOLD': 30.0,  # Threshold for very high speed (~65 mph)
    'CONSERVATIVE_PERSONALITY_FACTOR': 0.8,  # Factor for conservative personality
    'AGGRESSIVE_PERSONALITY_FACTOR': 1.1,  # Factor for aggressive personality when conditions allow
    'CONSERVATIVE_CURVE_FACTOR': 0.85,  # Factor for curve conservatism
    'CONSERVATIVE_GRADE_FACTOR': 0.9,  # Factor for grade conservatism
    'CONSERVATIVE_LOW_CONFIDENCE_FACTOR': 0.75,  # Factor for low confidence conservatism
    'CONSERVATIVE_VISIBILITY_FACTOR': 0.8,  # Factor for poor visibility conservatism
    'CURVE_FOLLOWING_DISTANCE_FACTOR': 1.5,  # Factor for increasing following distance in curves
    'GRADE_FOLLOWING_DISTANCE_FACTOR': 1.2,  # Factor for increasing following distance on grades
    'LOW_CONF_FOLLOWING_DISTANCE_FACTOR': 1.4,  # Factor for low confidence following distance
    'VISIBILITY_FOLLOWING_DISTANCE_FACTOR': 1.3,  # Factor for poor visibility following distance
    'HIGH_SPEED_FOLLOWING_DISTANCE_FACTOR': 1.2,  # Factor for high speed following distance
    'CONSERVATIVE_FOLLOWING_DISTANCE_FACTOR': 1.4,  # Factor for conservative following distance
    'AGGRESSIVE_FOLLOWING_DISTANCE_FACTOR': 0.85,  # Factor for aggressive following distance when conditions allow
    'SHARP_CURVE_THRESHOLD': 0.01,  # Threshold for sharp curve detection
    'BASE_FOLLOW_TIME': 1.5,  # Base follow time in seconds
    'MIN_FOLLOW_DISTANCE': 30.0  # Minimum follow distance
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
    'FAILURE_COUNT_THRESHOLD': 5  # Number of consecutive failures before disengagement
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
    assert 1.0 <= ADAPTIVE_BEHAVIOR_PARAMS['AGGRESSIVE_PERSONALITY_FACTOR'] <= 1.5, \
        "AGGRESSIVE_PERSONALITY_FACTOR must be between 1.0 and 1.5"

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