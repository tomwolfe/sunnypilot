# Integration Test Plan for sunnypilot 80/20 Improvements

## Objective
To validate the integrated behavior of the enhanced `PerformanceMonitor` and `SafetyMonitor` components within the `controlsd` pipeline, ensuring proper parameter adaptation, safety interventions, and environmental awareness across various realistic and edge-case scenarios.

## General Approach

Integration tests should focus on end-to-end behavior of the `controlsd` process. This involves:
1.  **Input Data Generation/Acquisition**: Providing realistic (mocked or recorded) data for all `SubMaster` inputs (`modelV2`, `carState`, `radarState`, `livePose`, `gpsLocationExternal`, `driverMonitoringState`, etc.).
2.  **Execution Environment**: Running `controlsd` in a controlled, non-real-time environment where inputs can be precisely controlled and outputs captured.
3.  **Output Validation**: Asserting on the state of `controlsd` (e.g., `performance_monitor` parameters, `safety_monitor` flags like `requires_intervention`, `safety_degraded_mode`, and the resulting `carControl` outputs like `actuators.accel` or disengagement events).

## Specific Test Scenarios

### 1. Lateral Error Metric Validation
*   **Scenario**: Simulate a vehicle consistently deviating laterally from the desired path.
*   **Input**:
    *   `modelV2.position.y[0]` representing a desired lateral position (e.g., 0.5m).
    *   `carState` with `vEgo`.
    *   `actual_state['lateral']` should be `0.0` as per the fix.
*   **Expected Outcome**: `PerformanceMonitor` should correctly calculate `lateral_accuracy` in meters. If the error is sustained above thresholds, parameter adaptation should be triggered (unless safety-locked).

### 2. Tunnel Entry/Exit Detection
*   **Scenario**: Simulate driving into and out of a tunnel.
*   **Input**:
    *   `modelV2` showing low lane line strength/confidence.
    *   `livePose` with consistent IMU data (smooth road).
    *   `gpsLocationExternal` transitioning from strong signal to weak/lost signal and back.
    *   `carState` with normal driving conditions.
*   **Expected Outcome**: `SafetyMonitor.in_tunnel` should transition to `True` upon entry and `False` upon exit. `lighting_condition` should reflect "dark" or "unknown" during tunnel passage. Safety score adjustments related to environmental conditions should occur.

### 3. Sustained Performance Degradation
*   **Scenario**: Simulate a prolonged period where performance metrics (e.g., lateral accuracy, longitudinal accuracy, ride comfort) are consistently below baselines.
*   **Input**:
    *   `PerformanceMonitor.evaluate_performance` receiving inputs that consistently yield poor metrics.
    *   `combined_degraded_mode` remains `True` for an extended duration.
*   **Expected Outcome**: `controlsd.py` should trigger `requires_intervention = True` and disengage after `performance_degradation_consecutive_frames` and `performance_degradation_disengage_time` thresholds are met. The `degradation_consecutive_count` should increment appropriately.

### 4. Intermittent Performance Degradation
*   **Scenario**: Simulate short, intermittent periods of performance degradation, where `combined_degraded_mode` fluctuates between `True` and `False`.
*   **Input**:
    *   `PerformanceMonitor.evaluate_performance` inputs fluctuate.
*   **Expected Outcome**: `controlsd.py` should *not* trigger disengagement if the degradation is not sustained beyond the configured consecutive frames and time thresholds. The `degradation_consecutive_count` should reset when `combined_degraded_mode` becomes `False`.

### 5. Safety Lockout for Parameter Adaptation
*   **Scenario**: Simulate a situation where performance adaptation is desired (`adapt_needed` is `True`), but the `overall_safety_score` is in a high-risk state.
*   **Input**:
    *   `PerformanceMonitor.should_adapt_parameters()` returns `True`.
    *   `SafetyMonitor.overall_safety_score` is below `self.safety_high_risk_threshold`.
*   **Expected Outcome**: `controlsd` should log a "Safety lockout active" warning, and `performance_monitor.tuning_params` should *not* be updated.

### 6. Sensor Staleness and Confidence Decay
*   **Scenario**: Simulate one or more critical sensor inputs becoming stale (not updating for an extended period).
*   **Input**:
    *   `mono_time` for a specific sensor (e.g., `modelV2_mono_time`, `radar_state_mono_time`) stops updating or updates very slowly.
*   **Expected Outcome**: The corresponding `self.SENSOR_healthy` flag in `SafetyMonitor` should become `False`. The `self.SENSOR_confidence` should decay over time, and the `overall_safety_score` should be reduced accordingly, potentially leading to intervention.

## Recommended Tools and Frameworks

*   **Openpilot Log Replay**: Utilize `tools/lib/logreader.py` or similar internal tools to replay recorded driving logs. Logs can be modified to inject specific conditions (e.g., simulate GPS loss, alter model outputs).
*   **Unit Test Frameworks**: For specific component-level integration, existing Python unit test frameworks (e.g., `pytest`, as seen in `test_improvements_validation.py`) can be extended.
*   **Mocking Libraries**: `unittest.mock` can be used to mock external dependencies (e.g., `Params`, `messaging`) for isolated testing of `controlsd` components.
*   **Simulation Environments**: For high-fidelity testing, an openpilot-compatible simulator would be ideal, though this is a more complex undertaking.

## Test Data Management
*   Maintain a suite of recorded driving logs covering diverse conditions (day/night, rain/clear, tunnels, various road types).
*   Create synthetic log segments or use log manipulation tools to induce specific failure modes or edge cases (e.g., sudden drop in model confidence, intermittent sensor data).

## Reporting
*   Each integration test should clearly state the scenario, inputs, and expected outcomes.
*   Test failures should provide detailed logs and state snapshots to aid in debugging.

By following this plan, developers can systematically verify the robustness and correctness of the new features.
