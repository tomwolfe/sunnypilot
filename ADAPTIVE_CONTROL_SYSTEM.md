# Adaptive Control System Documentation

## Overview
This system implements advanced adaptive control features for the openpilot control system, including multi-layered adaptive control, sensor fusion, and intelligent system optimization. This document describes the architecture and safety measures implemented to address complexity concerns.

## Architecture Components

### 1. Context-Aware Adaptive Gains
- Calculates driving context based on vehicle state, road conditions, and environment
- Adjusts PID controller gains based on multiple factors (speed, thermal state, traffic, weather)
- Includes validation to ensure gains remain within safe bounds

### 2. Radar-Camera Fusion
- Implements Kalman filter-based sensor fusion for lead vehicle detection
- Combines radar measurements with vision model outputs for improved accuracy
- Includes reliability scoring and validation of fused data

### 3. Vision Model Execution
- The vision model now *always runs* to prioritize safety and eliminate unreliable skipping logic.
- Previous optimizations based on system load and scene complexity were removed due to safety concerns and unreliability.
- This ensures continuous, real-time perception for critical driving functions.

### 4. Hardware Thermal Management
- Thermal bands for system components are *dynamically configured* based on hardware-specific throttling thresholds (e.g., for Snapdragon 845 SoC).
- Dynamic GPU/memory governor configuration, including adaptive switching between 'ondemand' and 'performance' based on thermal state and critical driving situations.
- Custom throttling parameters are now used to define the dynamic thermal bands.

## Safety Measures and Risk Mitigation

### Circuit Breaker System
The system implements an enhanced circuit breaker pattern to prevent cascading failures:

- **Adaptive Gains Circuit Breaker**: Reduced to 3 errors (was 5), 10-second cooldown (was 5s) with root cause analysis
- **Radar-Camera Fusion Circuit Breaker**: 3 errors, 15-second cooldown (was 10s) with root cause analysis
- **Vision Model Circuit Breaker**: Reduced to 5 errors (was 10), 45-second cooldown (was 30s) with root cause analysis

When triggered, circuit breakers:
- Disable the problematic feature
- Fall back to conservative, safe default behavior
- Include root cause analysis to identify systematic issues
- Prevent rapid resets by requiring stable operation period
- Log detailed error information for debugging

### Validation and Safety Checks
- Comprehensive gain validation with bounds checking
- NaN/Infinity detection and correction
- Sudden change smoothing to prevent control instability
- Physical plausibility validation for all sensor data
- Gain ratio validation to prevent integral windup

### Error Handling
- Graceful fallback to default values when errors occur
- Extensive logging for debugging and analysis
- Isolated error handling to prevent system-wide failures

### Additional Safety Improvements
- **Conservative Fallback Gains**: Default gains reduced from (0.5, 0.05, etc.) to (0.3, 0.03, etc.) for enhanced safety
- **Fixed Curvature Dependency**: Resolved missing self.curvature reference by calculating curvature directly from vehicle state
- **Improved Longitudinal Gain Extraction**: Added safe fallback when adaptive gains structure is unexpected
- **Adaptive GPU Management**: Intelligent switching between ondemand (thermal safety) and performance (critical situations) governors
- **Root Cause Analysis**: Circuit breakers now track error patterns to identify systematic issues and cascade failures

## Debugging and Telemetry

### Logging
- Detailed logging of driving context calculations
- Adaptive gains values and contributing factors
- Fusion process and reliability metrics
- System state and thermal information
- Error conditions and recovery actions

### Telemetry
- Enhanced controls state with contextual information
- Real-time gain values for analysis
- System performance metrics
- Safety system status

## Risk Mitigation Strategies

1. **Complexity Management**: Modular design separates concerns, enhanced circuit breakers provide isolation with root cause analysis
2. **Debugging Support**: Comprehensive logging at all system levels
3. **Heuristic Management**: Validation and bounds checking on all parameters
4. **Cascading Failure Prevention**: Enhanced circuit breaker system with reduced error tolerance and pattern analysis
5. **Performance Overhead**: Optimized algorithms and adaptive thermal management
6. **Test Coverage**: Unit tests for all major components
7. **Safety-First Design**: Conservative fallback behaviors and reduced gain margins for critical situations
8. **Thermal Performance Balance**: Adaptive governor switching that temporarily boosts performance during critical operations while maintaining overall thermal safety

## Performance Considerations

- Optimized algorithms to minimize computational overhead
- Caching mechanisms to reduce redundant calculations
- Adaptive execution based on system load and criticality
- Temporal consistency to maintain smooth control behavior

## Maintenance Guidelines

### When Modifying Adaptive Gains
- Always validate changes against safety bounds
- Test thoroughly across different driving conditions
- Update corresponding validation logic if needed

### When Modifying Sensor Fusion
- Ensure fallback behavior maintains safety
- Test with various sensor failure modes
- Verify reliability calculations remain accurate

### When Adding New Features
- Implement appropriate circuit breaker protection
- Add comprehensive validation and error handling
- Include detailed logging for debugging
- Consider impact on overall system complexity