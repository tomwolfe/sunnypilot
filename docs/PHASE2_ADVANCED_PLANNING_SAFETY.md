# Phase 2: Advanced Planning & Safety - Documentation

## Overview
Phase 2 enhances the sunnypilot system with advanced planning and comprehensive safety validation systems. This implementation integrates validation metrics from Phase 1 into control decisions and implements dynamic safety margins based on real-time confidence scores.

## Implementation Details

### 1. Enhanced Vision Integration in Controls
- **File Modified**: `selfdrive/controls/controlsd.py`
- **Integration**: Added subscription to `validationMetrics` topic
- **Functionality**: Controls now receive validation metrics from enhanced vision processor
- **Safety Validation**: Implemented multi-level validation state based on confidence thresholds

### 2. Dynamic Safety Margins for Lane Changes
- **Confidence Thresholds**: Lane changes require higher confidence (>0.7) compared to general operation (>0.6)
- **Behavior**: When confidence is low, lane changes are prevented or interrupted
- **Implementation**: Modified lane change state transitions based on validation metrics

### 3. Safety Validation Layers
- **Fallback Behaviors**: Triggered when overall confidence falls below 0.4
- **Degraded Mode**: Activated when confidence is between 0.4-0.6
- **System Alerts**: Generated when confidence falls below 0.75
- **Disengagement**: Automatic disengagement when critical thresholds are breached

### 4. Enhanced Longitudinal Planning
- **File Created**: `selfdrive/controls/lib/enhanced_longitudinal_planner.py`
- **Trajectory Smoothing**: Implemented exponential smoothing for lead vehicle state estimation
- **Probabilistic Spacing**: Dynamic safety distance calculation based on confidence and speed
- **Confidence-Based Limits**: Adjusted acceleration limits based on validation metrics

## New Validation Metrics and Thresholds

### Confidence Thresholds
| Threshold | Value | Behavior |
|-----------|-------|----------|
| Critical Safety | 0.4 | System disengagement |
| Degraded Mode | 0.6 | Conservative operation |
| Lane Change | 0.7 | Allow lane change operations |
| Lane Change Enhanced | 0.7+ for both overall and lane | Normal lane change behavior |

### Validation Metric Fields
| Field | Description |
|-------|-------------|
| `leadConfidenceAvg` | Average confidence of lead vehicle detection |
| `leadConfidenceMax` | Maximum confidence of lead vehicle detection |
| `laneConfidenceAvg` | Average confidence of lane detection |
| `overallConfidence` | Combined confidence score of all systems |
| `isValid` | Boolean indicating if system is functioning properly |
| `confidenceThreshold` | Threshold value for validity check |

## Safety Check Logic and Fallback Behavior

### Fallback Triggers
1. **Critical Fallback**: Overall confidence < 0.4 → Disengagement
2. **Conservative Control**: Lead confidence < 0.6 → Reduced PID limits
3. **Lane Change Prevention**: Combined confidence < 0.7 → Lane changes restricted
4. **System Alert**: Any metric below warning threshold → Alert raised

### Control Adjustments Based on Confidence
- **Low Lead Confidence**: PID limits reduced to 80% of normal
- **Low Lane Confidence**: Desired curvature reduced by 30% 
- **High Confidence**: Slight increase (10%) in responsiveness
- **Critical Low Confidence**: System disengages for safety

### Longitudinal Planning Adjustments
- **Smoothed Trajectory Estimation**: Lead distance and speed estimates are smoothed
- **Probabilistic Spacing**: Dynamic safe distance based on current speed and confidence
- **Conservative Acceleration**: More conservative acceleration targets when confidence is low
- **Adaptive Limits**: Acceleration limits adjusted based on confidence level

## Hardware Compliance
All implementations adhere to Comma Three constraints:
- **RAM Usage**: < 1.4 GB - Additional objects are efficiently managed with pre-allocation where possible
- **CPU Usage**: < 5% average, < 10% peak - Optimized algorithms with minimal computational overhead
- **End-to-end Latency**: < 80 ms - Real-time processing maintained through efficient validation checks

## Message Schema Changes
- **New Topic**: `validationMetrics` - Publishes validation metrics from enhanced vision
- **Enhanced ControlsState**: Added validation metrics and safety status fields to ControlsState
- **Backward Compatibility**: All new fields are optional to maintain compatibility

## Integration Points
- **Modeld Integration**: Enhanced vision metrics published to validationMetrics topic
- **Controls Integration**: Validation metrics consumed in controlsd.py for decision making
- **Longitudinal Control**: Enhanced planner integrated into longitudinal control loop
- **Safety System**: Multi-level safety validation added throughout control pipeline

## Testing
- **Unit Tests**: Comprehensive unit tests for EnhancedLongitudinalPlanner
- **Integration Tests**: Tests for validation metrics integration and safety systems
- **Edge Case Tests**: Verification of behavior under low confidence conditions
- **Regression Tests**: Ensured existing functionality remains intact

## Performance Considerations
- **Efficient Computation**: Validation checks use optimized comparisons
- **Memory Management**: Pre-allocated arrays for repeated operations
- **Real-time Processing**: All safety checks integrated within existing control loops
- **Minimal Overhead**: Additional computational cost is kept to minimum for real-time performance