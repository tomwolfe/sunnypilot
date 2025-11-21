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
- Curvature values must not exceed 0.1 m<sup>-1</sup> (default limit)

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