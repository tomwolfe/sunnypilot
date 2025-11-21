# Curvature Gain Feature - Safety Analysis

## Overview
This document provides a formal safety analysis for the curvature gain feature in openpilot's lateral control system. The feature adjusts the proportional gain based on the curvature of the road to provide more responsive steering during sharp turns while maintaining stability during straight-line driving.

## Hazard Analysis

### Identified Hazards

| Hazard | Potential Cause | Safety Mitigation |
|--------|----------------|-------------------|
| Excessive steering gain leading to oscillations or instability | High curvature gain combined with high speed-based gain | `maxCurvatureGainMultiplier` parameter limits total gain multiplication |
| Noise amplification causing jerky steering | Noisy `modeld` curvature estimates | 0.1s `FirstOrderFilter` applied to smooth desired curvature |
| Parameter misconfiguration causing unsafe behavior | Malformed or invalid `CurvatureGainInterp` values | Comprehensive validation in `helpers.py` |
| Physical limit violation | Curvature values exceeding safe turning capability | Dynamic `MaxCurvatureForGainInterp` based on vehicle characteristics |
| Oscillation detection and damping | No real-time oscillation detection | (To be implemented - future enhancement) |

### Risk Assessment

| Risk | Probability | Severity | Risk Level | Mitigation Status |
|------|-------------|----------|------------|-------------------|
| System instability due to excessive gain | Low | High | Medium | Implemented |
| Jerky steering due to noisy inputs | Medium | Medium | Medium | Implemented |
| Invalid configuration causing erratic behavior | Low | High | Medium | Implemented |
| Oscillations not detected in real-time | Medium | Medium | Medium | Planned |

## Safety Mechanisms

### Primary Safety Mechanisms

1. **Configurable Maximum Multiplier (`maxCurvatureGainMultiplier`)**
   - Prevents explosive gain multiplication when both speed-based and curvature-based gains are high
   - Vehicle-specific defaults based on mass (3.0 for >2000kg, 4.0 for standard, 5.0 for <1200kg)
   - Range limited to [1.0, 10.0] to prevent unsafe values

2. **Input Validation**
   - Curvature values must be non-negative
   - Curvature values must be in ascending order
   - Gain multipliers must be ≥ 1.0
   - Both arrays must have matching lengths
   - Comprehensive validation in `helpers.py`

3. **Curvature Value Clamping**
   - Maximum curvature limit dynamically calculated based on vehicle characteristics (wheelbase, steering ratio, center of gravity, mass)
   - Customizable via `MaxCurvatureForGainInterp` parameter with vehicle-specific safe bounds
   - Prevents consideration of physically unrealistic turn radii

4. **Signal Filtering**
   - 0.1s first-order filter applied to `desired_curvature` to reduce noise
   - Prevents jerky steering adjustments due to noisy model outputs

### Secondary Safety Mechanisms

1. **Default Fallback Behavior**
   - Safe default values used on validation failure: `[[0.0], [1.0]]` for curvature gain
   - Default `maxCurvatureGainMultiplier` of 4.0 if not configured

2. **Runtime Gain Limiting**
   - Maximum allowable gain calculated as `original_k_p * maxCurvatureGainMultiplier`
   - Effective gain clamped to prevent exceeding safety limits

## Failure Modes and Effects Analysis (FMEA)

| Failure Mode | Root Cause | Effect | Detection | Mitigation |
|--------------|------------|--------|-----------|------------|
| Parameter validation failure | Malformed JSON input | System uses safe defaults | Validation logic in `helpers.py` | Defaults to `[[0.0], [1.0]]` |
| Filter malfunction | FirstOrderFilter failure | Noisy steering commands | No specific detection | Manual testing, logging |
| Excessive gain | Misconfigured parameters | Oscillations, instability | `maxCurvatureGainMultiplier` limit | Gain clamping in PID controller |
| Invalid curvature values | Modeld output errors | Erratic steering | Clamping in `helpers.py` | Dynamic limit based on vehicle characteristics |

## Safety Requirements

### Functional Safety Requirements

1. **SR-1**: The curvature gain must not exceed the configurable maximum multiplier under any operating condition
2. **SR-2**: All curvature gain parameters must be validated before use in the control system
3. **SR-3**: The system must handle invalid parameter configurations gracefully by using safe defaults
4. **SR-4**: The desired curvature signal must be filtered to reduce noise before gain computation

### Performance Safety Requirements

1. **PR-1**: The system shall detect and prevent oscillations that may result from high gain settings
2. **PR-2**: Steering response shall remain stable across the full range of valid curvature values
3. **PR-3**: The system shall maintain control authority within physical limits of the vehicle

## Verification and Validation

### Testing Strategy

1. **Unit Testing**: Comprehensive test coverage for validation logic in `test_helpers.py`
2. **Integration Testing**: End-to-end testing in `test_latcontrol_pid_curvature.py`
3. **Benchmark Testing**: Stress testing across multiple vehicle models using `curvature_gain_benchmark.py`

### Test Coverage

- Parameter validation edge cases
- Filter behavior under various noise conditions  
- Gain limiting under extreme conditions
- Vehicle-specific defaults and limits

## Future Safety Enhancements

### Recommended Improvements

1. **Adaptive Damping**: Implementation of real-time oscillation detection and damping
2. **Enhanced Monitoring**: Additional metrics and logging for gain-related issues
3. **Safety Case Update**: Periodic review and update of safety analysis based on field experience

## Conclusion

The curvature gain feature implements multiple layers of safety mechanisms to prevent unsafe behavior while enabling improved steering performance. The primary safety mechanisms (gain limiting, input validation, signal filtering) have been implemented and tested. The secondary mechanisms (default fallbacks, runtime limiting) provide additional safety margins. The identified risk of lacking real-time oscillation detection is planned for future implementation.

The safety analysis demonstrates that the feature is appropriately designed for safe operation when configured correctly, with multiple fail-safe mechanisms in place.