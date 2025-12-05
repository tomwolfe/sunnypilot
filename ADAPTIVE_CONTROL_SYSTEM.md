# Adaptive Control System (ACS) for Sunnypilot

## Overview
The Adaptive Control System (ACS) is a sophisticated, multi-layered feedback loop that replaces a simple PID controller with a dynamic, context-aware safety engine. This system integrates several critical new components to enhance safety and reliability.

## Components

### 1. DrivingContextAnalyzer
Synthesizes data from `carState`, `modelV2`, and `radarState` to determine real-time driving context (curvy road, traffic density, weather, time of day).

### 2. AdaptiveGainsController
Dynamically calculates `steer_kp/ki/kd` and `accel_kp/ki` based on vehicle speed, thermal state, and the detected driving context. This ensures control aggressiveness is always appropriate to the situation.

### 3. CircuitBreakerManager
Monitors for recurring or rapid errors. If a subsystem (e.g., sensor fusion, adaptive gain calculation) fails repeatedly, it triggers a "circuit breaker," disabling the faulty component and forcing the system into a conservative fallback mode. This prevents cascading failures.

### 4. EdgeCaseHandler
Detects unusual scenarios (e.g., sensor fusion failures, extreme conditions) and applies specific, pre-defined control modifications to maintain safety.

### 5. SafeSelfLearningManager
A sophisticated, safety-wrapped learning system. It compares the model's predicted curvature (`desired_curvature`) against the *actual* vehicle curvature (calculated from `steeringAngleDeg` and `wheelbase`). It learns from this error and adjusts future predictions, but crucially, it does so within a safety framework that prevents over-adaptation.

### 6. SafetyManager
The central authority. It performs final, holistic safety checks on all control outputs and can trigger an `IMMEDIATE_DISABLE` if any critical threshold is breached.

## Safety Features

### Thermal Management
The system has a sophisticated `ThermalManager` that dynamically throttles the model's execution rate based on real-time thermal sensor data (`thermal_throttle_factor`). The critical logic `min(thermal_throttle_factor, model_param_throttle_factor)` ensures that **hardware safety always overrides any user or model setting**. A `ThermalStatus.danger` state triggers an `IMMEDIATE_DISABLE`.

### Parameter Validation & Bounds Checking
Every critical parameter (`steer_kp`, `accel_ki`, etc.) is rigorously validated. Out-of-bounds values are clamped, and warnings are logged. Non-finite values (`NaN`, `inf`) are detected and replaced with safe defaults, preventing catastrophic control commands.

### Circuit Breaker Rigor
The circuit breaker system is designed to be aggressive. It doesn't just count errors; it analyzes patterns (repeated errors, rapid sequences) to detect systematic issues and trigger a permanent disable, prioritizing safety over continuous operation.

## System Architecture

The system now has a clear, hierarchical safety protocol:
**Hardware thermal safety > Circuit breaker isolation > Conservative fallback gains > Adaptive control**

This layered approach is the gold standard for safety-critical systems.

## Risk Mitigation

### Complexity Management
- Modular design separates concerns
- Enhanced circuit breakers provide isolation with root cause analysis
- Comprehensive logging at all system levels

### Over-Adaptation Prevention
- Learning system detects and prevents over-adaptation patterns
- Validation and bounds checking on all parameters
- Sudden change smoothing to prevent control instability
- Physical plausibility validation for all sensor data

### Performance Considerations
- Optimized algorithms to minimize computational overhead
- Caching mechanisms to reduce redundant calculations
- Adaptive execution based on system load and criticality
- Temporal consistency to maintain smooth control behavior

## Testing and Validation

The system includes comprehensive unit tests and integration tests to validate the complex new components, ensuring the system works as designed.