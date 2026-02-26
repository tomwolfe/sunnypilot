# Direct Torque E2E Implementation

## Overview

This implementation adds **Direct Torque End-to-End (E2E)** control to sunnypilot, allowing the model to output raw torque commands directly, bypassing the traditional curvature→PID chain. This is a critical step toward true Level 3 autonomy.

## Architecture Change

### Before (Traditional)
```
Image → Model → Desired Curvature → PID Controller → Torque Command
```

### After (Direct Torque E2E)
```
Image + IMU + History → Model → Raw Torque Command
```

## Files Modified

### 1. Model Output Constants
- **`sunnypilot/modeld_v2/constants.py`**: Added `DIRECT_TORQUE_WIDTH = 1`
- **`sunnypilot/models/split_model_constants.py`**: Added `DIRECT_TORQUE_WIDTH = 1`

### 2. Model Output Parsing
- **`sunnypilot/modeld_v2/parse_model_outputs_split.py`**: Added parsing for `direct_torque` output

### 3. Lateral Control Extension
- **`sunnypilot/selfdrive/controls/lib/latcontrol_torque_ext.py`**: 
  - Added `DirectTorqueE2E` parameter check
  - Added `direct_torque_blend` for smooth transitions
  - Added `_apply_direct_torque()` method
  - Added `_update_direct_torque_blend()` for safety blending

### 4. Control Interface Updates
- **`selfdrive/controls/lib/latcontrol.py`**: Added `model_v2` parameter to abstract `update()` method
- **`selfdrive/controls/lib/latcontrol_torque.py`**: Pass `model_v2` to extension
- **`selfdrive/controls/lib/latcontrol_pid.py`**: Added `model_v2` parameter (unused)
- **`selfdrive/controls/lib/latcontrol_angle.py`**: Added `model_v2` parameter (unused)
- **`sunnypilot/selfdrive/controls/lib/latcontrol_torque_v0.py`**: Pass `model_v2` to extension
- **`selfdrive/controls/controlsd.py`**: Pass `model_v2` from message bus to `LaC.update()`

## Key Features

### 1. Smooth Blending
The implementation includes a smooth blending mechanism between traditional and direct torque control:
- **Blend rate**: 2% per 20ms step (~0.5 second transition)
- **Low-speed cutoff**: Direct torque disabled below 5 m/s for safety
- **Fallback**: Automatically falls back to traditional control if model output unavailable

### 2. Safety Limits
- Torque output is clamped to `steer_max` (same as traditional control)
- Low-speed disable prevents erratic behavior during parking/stop-and-go
- Blend factor ensures no sudden transitions

### 3. Monitoring
- `direct_torque_available`: Flag indicating model is outputting direct_torque
- `direct_torque_blend`: Current blend factor (0.0 = traditional, 1.0 = direct)
- PID log is updated even in direct mode for debugging

## Usage

### Enable Direct Torque E2E
```bash
# Set the parameter to enable direct torque mode
oath set DirectTorqueE2E 1
```

### Model Requirements
The model must output a `direct_torque` tensor with shape `[1, 1]` containing the raw torque command in the range `[-1.0, 1.0]`.

### Training Considerations
For best results, the model should be trained with:
- **Input**: Images, IMU data, vehicle state, temporal history
- **Output**: `direct_torque` (raw torque command)
- **Loss**: MSE between predicted torque and logged torque (from human drivers or expert policy)

## Implementation Details

### `_apply_direct_torque()` Method
```python
def _apply_direct_torque(self, model_v2, CS):
    # Extract direct torque from model output (shape: [1, 1])
    self._direct_torque_output = float(model_v2.direct_torque[0, 0])
    
    # Apply blending with traditional control for safety
    if self.direct_torque_blend < 1.0 and self._pid_log is not None:
      traditional_torque = self._pid_log.output
      self._output_torque = (self.direct_torque_blend * self._direct_torque_output + 
                            (1.0 - self.direct_torque_blend) * traditional_torque)
    else:
      self._output_torque = self._direct_torque_output
    
    # Apply safety limits
    self._output_torque = float(max(-self.lac_torque.steer_max, 
                                   min(self.lac_torque.steer_max, self._output_torque)))
```

### `_update_direct_torque_blend()` Method
```python
def _update_direct_torque_blend(self, CS):
    target_blend = 1.0 if (self.direct_torque_enabled and self.direct_torque_available) else 0.0
    
    # Blend over 0.5 seconds for smooth transitions
    blend_rate = 0.02  # ~2% per 20ms step
    if target_blend > self.direct_torque_blend:
      self.direct_torque_blend = min(target_blend, self.direct_torque_blend + blend_rate)
    else:
      self.direct_torque_blend = max(target_blend, self.direct_torque_blend - blend_rate)
    
    # Disable direct torque at very low speeds (< 5 m/s) for safety
    if CS.vEgo < 5.0:
      self.direct_torque_blend = 0.0
```

## Next Steps

### Step 2: Expand Temporal Memory
- Increase `DrivingModelFrame.buffer_length` from 2→20
- Add attention-based temporal fusion
- Leverage existing 99-frame `features_buffer` more effectively

### Step 3: Learned Vehicle Dynamics
- Feed `liveLocationKalman` (yaw_rate, accel) as model input
- Remove dependency on `steerRatio` and `wheelbase` from `values.py`
- Let model infer vehicle mass and steering ratio from observed dynamics

### Step 4: World Models for Negotiation
- Add `lane_change_probability` and `cost_map` outputs
- Replace hardcoded `lane_turn_desire.py` logic with learned cost function
- Visualize probability heatmaps for negotiation scenarios

## Testing

### Unit Tests
- Verify blend transitions are smooth (no torque spikes)
- Verify low-speed cutoff works correctly
- Verify fallback to traditional control when `direct_torque` unavailable

### Integration Tests
- Test on closed course with gradual blend-in
- Log `direct_torque_blend`, `_direct_torque_output`, and traditional torque for comparison
- A/B test against traditional curvature mode

## References
- Original analysis: "Final Grade: A- (Advanced Hybrid E2E)" document
- NNFF implementation: `sunnypilot/selfdrive/controls/lib/nnlc/`
- Model output parsing: `sunnypilot/modeld_v2/parse_model_outputs_split.py`
