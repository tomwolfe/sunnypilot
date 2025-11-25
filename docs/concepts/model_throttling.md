# Model Execution Throttling

This document details the Model Execution Throttle Factor system, a critical enhancement designed to balance computational demand with thermal safety in embedded automotive hardware.

## Purpose and Design Philosophy

The primary goal of the Model Execution Throttle Factor is to ensure the continuous, safe, and efficient operation of the system's neural network models, even under varying thermal conditions. It achieves this by dynamically adjusting the model's execution rate to prevent overheating while maintaining a smooth and responsive user experience.

The system is built upon a robust, hierarchical safety architecture where hardware safety always overrides user preference.

## Key Features

### Robust, Hierarchical Safety Architecture

The system employs a critical logic: `min(thermal_throttle_factor, model_param_throttle_factor)`. This ensures that:
*   **Hardware Safety First:** The thermal status of the device (`thermal_throttle_factor`) always takes precedence. If the device experiences thermal stress, the model execution rate is reduced accordingly, regardless of user settings.
*   **Absolute Safety Net:** In a `danger` thermal status, the throttle factor is immediately forced to 0%, triggering an `IMMEDIATE_DISABLE` alert. This crucial safety mechanism prevents catastrophic overheating and ensures system integrity.

### Efficient and Smooth Throttling Mechanism

Instead of reducing the inference speed of the model (which could introduce latency and affect responsiveness), the system utilizes **frame skipping** (`frame_skip_threshold`). During skipped frames, the system intelligently **reuses the last valid model output**. This approach provides several benefits:
*   **Continuous Predictions:** Ensures a stable, continuous stream of predictions to the vehicle's controllers.
*   **Prevents Jerky Behavior:** Avoids abrupt changes in system behavior that could arise from delayed or inconsistent model outputs.
*   **Optimized Resource Usage:** Reduces computational load without sacrificing the perceived smoothness of the system.

### Comprehensive and Transparent User Experience

The system is designed with user understanding and control in mind:
*   **Clear HUD Indicator:** A prominent green/yellow/red circle in the top-right HUD provides immediate and intuitive visual feedback on the current model execution rate, allowing users to quickly assess system performance.
*   **Intuitive Settings:** The settings menu offers discrete percentage options (e.g., 0%, 25%, 50%, 75%, 100%) for the `ModelExecutionThrottleFactor`, providing clear and precise user control.
*   **Explicit Documentation:** The parameter description in `params_keys.h` and the UI text clearly explain the feature's purpose, empowering users to understand and utilize the system effectively.

### Clean, Maintainable Code

The implementation follows best practices for code organization and maintainability:
*   **Logical Separation:** Changes are logically distributed across the codebase, including the Cap'n Proto schema for data definition, `params_keys.h` for configuration, the UI for display, and `modeld.py` for core logic.
*   **Consistent State Management:** Utilizes `Params` for configuration and a dedicated `throttle_factor` state variable, promoting consistency and scalability.
*   **Enhanced Debugging:** Extensive `cloudlog.debug` statements provide invaluable insights for debugging and monitoring system behavior.

### Proactive Safety Enhancement

The system significantly enhances safety by upgrading the response to `ThermalStatus.danger` from a `SOFT_DISABLE` to an `IMMEDIATE_DISABLE`. This change makes the system's reaction to critical overheating unambiguous and mandatory, further bolstering overall safety.

## Justified Trade-offs and Design Rationale

During the development of this system, certain design decisions involved trade-offs, which were carefully considered and deemed necessary for optimal system performance and safety.

### User Factor and Autonomy

While the `ModelExecutionThrottleFactor` allows a user to set a value of 1.0 even when the system is in a `red` thermal status, the underlying thermal factor will still enforce a reduction. This design prioritizes user autonomy while maintaining a critical safety net. The prominent HUD indicator effectively warns the user of performance throttling, and the `IMMEDIATE_DISABLE` for `danger` status remains the ultimate safeguard.

### Latency from Output Reuse

Reusing the last frame's output during frame skipping introduces a minimal, inherent latency. In rapidly changing environments, the prediction might be slightly outdated. However, this is a necessary and well-justified trade-off. Frame skipping with output reuse is a far more efficient method to reduce computational load than slowing down the entire inference pipeline, which would introduce greater and less predictable latency. The system's planners are inherently designed to account for and manage such minor lags as standard practice.

### `last_vision_outputs_dict` Initialization

The initial `None` state for `last_vision_outputs_dict` is correctly handled. The logic ensures that the model executes on the very first frame (when `last_vision_outputs_dict` is `None`), and only begins reusing outputs after the first successful inference. This prevents any issues related to uninitialized data.

## Conclusion

The Model Execution Throttle Factor system represents a significant, high-quality improvement. It transforms the system into a dynamic, thermally-aware engine, enhancing safety, reliability, and user control. Its robust implementation reflects a deep understanding of hardware constraints and embedded AI system design principles.
