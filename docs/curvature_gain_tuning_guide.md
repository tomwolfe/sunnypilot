# Curvature Gain Tuning Guide for openpilot

## Overview
The curvature gain feature enhances openpilot's lateral control by adjusting the proportional gain based on the curvature of the road. This allows for more responsive steering during sharp turns while maintaining stability during straight-line driving.

## Parameter Configuration

### Parameter Format
The curvature gain interpolation is configured as a JSON array with two sub-arrays:
```json
[[curvatures], [gain_multipliers]]
```

Where:
- `curvatures`: Array of absolute curvature values in meters<sup>-1</sup>
- `gain_multipliers`: Array of corresponding gain multipliers for the proportional term

### Example Configuration
```json
[[0.0, 0.02, 0.04, 0.06], [1.0, 1.2, 1.5, 2.0]]
```

This configuration means:
- At 0.0 m<sup>-1</sup> curvature (straight line): Use 1.0x the base proportional gain
- At 0.02 m<sup>-1</sup> curvature: Use 1.2x the base proportional gain
- At 0.04 m<sup>-1</sup> curvature: Use 1.5x the base proportional gain
- At 0.06 m<sup>-1</sup> curvature (16.7m radius turn): Use 2.0x the base proportional gain

## Physical Interpretation

Curvature values correspond to turn radii as follows:
- 0.01 m<sup>-1</sup> = 100m radius turn (very gentle curve)
- 0.02 m<sup>-1</sup> = 50m radius turn (gentle curve)
- 0.05 m<sup>-1</sup> = 20m radius turn (sharp curve) 
- 0.10 m<sup>-1</sup> = 10m radius turn (very sharp curve)

## Desired Curvature Source

The `desired_curvature` value, which is used by the lateral controller to determine the target steering angle, originates from openpilot's path planner (e.g., `modeld`). It represents the predicted curvature of the road ahead and is expressed in units of meters<sup>-1</sup>. This value is continuously updated based on sensor data and the vehicle's dynamics.

## Tuning Guidelines

### For Different Vehicle Types

#### Heavy Vehicles (SUVs, Trucks)
- Use more conservative gain multipliers
- Example: `[[0.0, 0.02, 0.04, 0.06], [1.0, 1.1, 1.3, 1.6]]`
- Higher mass requires more stable control to avoid oscillations

#### Sporty Vehicles (Lower center of gravity)
- Can handle more aggressive gain multipliers
- Example: `[[0.0, 0.02, 0.04, 0.06], [1.0, 1.3, 1.7, 2.3]]`
- Better handling characteristics allow for more responsive steering

#### Standard Passenger Cars
- Use balanced settings as default
- Example: `[[0.0, 0.02, 0.04, 0.06], [1.0, 1.2, 1.5, 2.0]]`
- Good compromise between responsiveness and stability

### Tuning Process

1. **Start Conservative**: Begin with lower gain multipliers than expected
2. **Test Gradually**: Increase multipliers in small increments (0.1-0.2)
3. **Monitor Behavior**: Watch for:
   - Overshooting in curves (gain too high)
   - Under-responsive steering (gain too low)
   - Oscillations or instability
4. **Validate Safety**: Ensure the system remains stable in all conditions

### Validation Requirements

The system enforces these validation rules:
- Curvature values must be non-negative
- Curvature values must be in ascending order
- Gain multipliers must be ≥ 1.0 (never reduce gain in curves)
- Both arrays must have matching lengths
- Curvature values are clamped to a maximum of 0.1 m<sup>-1</sup> (default) or a user-defined `CurvatureMaxLimit` (between 0.05 m<sup>-1</sup> and 0.2 m<sup>-1</sup>) if they exceed these limits. This prevents invalid input from causing system errors while allowing for reasonable adjustments.

## Setting the Parameter

The curvature gain interpolation can be set via the `CurvatureGainInterp` parameter in the openpilot parameter system.

Example via shell command:
```bash
p "CurvatureGainInterp" '[[0.0, 0.02, 0.04, 0.06], [1.0, 1.2, 1.5, 2.0]]'
```

## Performance Monitoring

Lateral control metrics are logged to `/data/openpilot/metrics/lateral_control_metrics.csv` for analysis and tuning. Use the provided plotting script to visualize steering performance:

```bash
python scripts/plot_lateral_metrics.py
```

## Troubleshooting

### Symptom: Vehicle oscillates during turns
**Solution**: Reduce gain multipliers, especially for higher curvature values

### Symptom: Steering feels unresponsive in curves
**Solution**: Gradually increase gain multipliers

### Symptom: Unstable at high curvature
**Solution**: Ensure gain multipliers don't exceed safe limits and verify the curve is within physical limits

## Balancing Speed-Based and Curvature-Based Gains

Tuning curvature gain and speed-based proportional gain (`kpV`) together is crucial for achieving optimal steering behavior across different driving scenarios. While curvature gain adapts steering stiffness to the sharpness of a turn, speed-based gain adjusts the overall steering response based on vehicle speed.

### Key Interactions

-   **High Speeds, Gentle Curves (e.g., Highway Driving):** At high speeds, `kpV` is typically lower to ensure stability and prevent overly aggressive steering. In gentle curves, `curvatureGain` will be low (or 1.0), so the steering feel is primarily determined by the speed-based gain. The goal is smooth, subtle corrections.
-   **Low Speeds, Sharp Turns (e.g., Urban Driving):** At low speeds, `kpV` is higher to provide more responsive steering. When entering a sharp turn, `curvatureGain` increases significantly, multiplying the already high proportional gain to provide the necessary torque to navigate the corner without understeering.

### Tuning Strategy Example

Consider a vehicle that feels sluggish in sharp turns but unstable at high speeds.

#### Desired Qualitative Outcomes:
-   **Smooth Turn-In**: The vehicle should initiate turns smoothly without abrupt steering inputs.
-   **Precise Path Following**: The vehicle should follow the desired path accurately without understeering or oversteering.
-   **No Oscillations**: The steering should remain stable, avoiding any back-and-forth oscillations, especially in curves.

#### Suggested Quantitative Metrics (from `lateral_control_metrics.csv`):
-   **Lateral Acceleration Error**: Aim for lateral acceleration error consistently below 0.1 m/s<sup>2</sup>.
-   **Steering Angle Error**: Keep the average steering angle error under 1-2 degrees in sustained curves.
-   **Oscillation Frequency/Amplitude**: Monitor for high-frequency, low-amplitude oscillations in steering angle, which indicate instability. Lower frequencies and amplitudes are generally preferred.

1.  **Address High-Speed Instability First:** Before touching curvature gain, tune the speed-based PID gains (`kpBP` and `kpV`) to be stable and comfortable for highway driving. This might involve lowering `kpV` at higher speeds.
2.  **Tune for Sharp Corners:** Once high-speed driving is satisfactory, focus on low-speed corners. If the car feels like it's understeering or not turning aggressively enough, increase the `curvatureGainInterp` multipliers for higher curvature values.

**Example Scenario: SUV Tuning**

An SUV might require a less aggressive setup.

-   **`kpV`:** `[1.2, 1.0, 0.8]` for `kpBP` at `[15, 25, 35]` m/s. This provides a stable response at highway speeds (35 m/s ≈ 78 mph).
-   **`curvatureGainInterp`:** `[[0.0, 0.02, 0.04, 0.06], [1.0, 1.1, 1.3, 1.6]]`. This provides a modest increase in gain for sharper turns, preventing the sluggish feel without making the steering feel overly artificial.

By tuning these two parameters iteratively, you can achieve a balanced steering response that is both stable on straightaways and responsive in turns. Always test changes in a safe environment.
