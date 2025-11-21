# Enhanced Safety Monitoring for sunnypilot

## Overview
This document describes the enhanced safety monitoring system for sunnypilot autonomous driving capabilities. The system provides multi-sensor fusion validation, confidence thresholds, environmental adaptation, and curve anticipation.

## Key Features Implemented

### 1. Multi-Sensor Fusion Validation
- Integrates camera, radar, and IMU data for comprehensive safety assessment
- Uses weighted confidence scoring based on environmental conditions
- Provides real-time safety score calculation

### 2. Model Confidence Monitoring
- Tracks neural network output confidence in real-time
- Applies filtering for smooth confidence transitions
- Sets safety thresholds for intervention

### 3. Environmental Condition Detection
- Improved lighting condition detection (night, dawn/dusk, tunnel, normal) with enhanced tunnel detection logic
- Multi-indicator weather condition estimation (clear, rain, snow, fog) using IMU, acceleration, and jerk data
- Advanced road condition assessment using IMU data with multiple indicator fusion

### 4. Curve Anticipation Enhancement
- Analyzes path curvature up to 200m ahead using proper path parameterization
- Calculates safe speed based on curvature and vehicle dynamics with improved accuracy
- Applies conservative driving when approaching curves at high speed

### 5. Lane Deviation Monitoring
- Tracks vehicle position relative to lane center
- Applies safety penalties for excessive deviation
- Maintains lane keeping confidence

### 6. Safety Threshold Validation
- Validates safety parameter thresholds to ensure they are within proper ranges (0.0-1.0)
- Ensures hierarchical threshold relationships (critical < high risk < moderate)
- Provides warnings for invalid parameter values

## Integration Points

### In controlsd.py:
- Added radarState to SubMaster for multi-sensor fusion
- Fixed polling list to include all messages checked in safety validation (modelV2, carState, carControl, radarState, livePose, selfdriveState)
- Integrated SafetyMonitor class for comprehensive safety assessment
- Added safety-degraded mode that reduces acceleration limits
- Applied safety-based interventions when required
- Added safety threshold parameter validation with range checks

## SafetyMonitor API Documentation

### Class: SafetyMonitor
The main class for the safety monitoring system.

#### Methods:
- `__init__()`: Initializes the safety monitor with default thresholds and filters.
- `update(model_v2_msg, radar_state_msg, car_state_msg, car_control_msg) -> Tuple[float, bool, Dict]`: Main update function that processes sensor inputs and returns safety assessment.
  - Parameters:
    - model_v2_msg: Cereal message with model outputs
    - radar_state_msg: Cereal message with radar data
    - car_state_msg: Cereal message with car state (velocity, acceleration, etc.)
    - car_control_msg: Cereal message with control data
  - Returns:
    - float: Overall safety score (0.0-1.0)
    - bool: Whether safety intervention is required
    - Dict: Detailed safety report with all metrics
- `update_model_confidence(model_v2_msg)`: Updates model confidence based on neural network outputs.
- `update_radar_confidence(radar_state_msg)`: Updates radar confidence based on lead detection reliability.
- `update_camera_confidence(model_v2_msg, car_state_msg)`: Updates camera confidence based on lane detection and visual clarity.
- `detect_environmental_conditions(model_v2_msg, car_state_msg, car_control_msg)`: Detects environmental conditions (lighting, weather, road).
- `detect_curve_anticipation(model_v2_msg, car_state_msg)`: Enhanced curve anticipation with improved safety margins.
- `calculate_overall_safety_score(car_state_msg)`: Calculates overall safety score based on all inputs.
- `evaluate_safety_intervention_needed(safety_score, car_state_msg)`: Determines if safety intervention is required.

## Safety Behaviors

### Normal Operation:
- Standard acceleration and lateral control parameters
- Full system functionality

### Safety Degraded Mode:
- Reduced acceleration and braking limits (70-85% of normal)
- Enhanced caution in curve anticipation
- More conservative lane keeping parameters

### Safety Intervention:
- Immediate conservative driving behavior
- Potential disengagement if safety score falls too low
- Clear logging of safety events

## Configuration Parameters

- `model_confidence_threshold`: 0.7 (minimum acceptable model confidence)
- `radar_confidence_threshold`: 0.6 (minimum acceptable radar confidence) 
- `lane_deviation_threshold`: 0.8m (maximum acceptable deviation from lane center)
- `max_anticipation_distance`: 200m (distance to look ahead for curve detection)

## Testing

The safety monitor has been tested for:
- Normal driving conditions
- Low model confidence scenarios
- Curve anticipation at various speeds
- Environmental condition detection
- Missing sensor data scenarios (e.g., livePose unavailable)
- Radar failure with unreliable lead detection
- Conflicting sensor data scenarios

## Performance Considerations

- Optimized for Comma 3X hardware (Qualcomm Snapdragon 845)
- Minimal computational overhead
- Efficient filtering using FirstOrderFilter
- Pre-allocated arrays to minimize memory allocation

## Future Enhancements

- Integration with weather APIs for more accurate weather detection
- Enhanced road surface condition detection
- Advanced multi-modal sensor fusion algorithms
- Machine learning-based safety assessment