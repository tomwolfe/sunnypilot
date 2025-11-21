# Curvature Gain Configuration

## Parameters

### CurvatureGainInterp
JSON array configuring the curvature-dependent gain interpolation for the PID controller.

Format: `[[curvatures], [gain_multipliers]]`
- `curvatures`: Array of absolute curvature values in m^-1
- `gain_multipliers`: Array of corresponding gain multipliers (≥ 1.0)

Example: `[[0.0, 0.02, 0.04, 0.06], [1.0, 1.2, 1.5, 2.0]]`

### MaxCurvatureGainMultiplier (Optional)
Float value defining the maximum allowed gain multiplier during high-curvature operations.
Used to prevent excessive gain multiplication when both speed-based and curvature-based gains are high.
Range: 1.0 to 10.0 (default: 3.0 for heavy vehicles, 4.0 for standard vehicles, 5.0 for light vehicles)
Default: Configured based on vehicle mass to provide appropriate safety margins
- Heavy vehicles (>2000kg): 3.0 (more conservative)
- Standard vehicles: 4.0 (balanced)
- Light vehicles (<1200kg): 5.0 (more responsive)

Example: `6.0` for high-performance applications requiring more aggressive response

### MaxCurvatureForGainInterp (Optional)
Float value defining the maximum allowed curvature value in m^-1.
Used to customize the physical limits for different driving environments.
Range: 0.05 to 0.2 m^-1 (5m to 20m radius turns)
Default: 0.1 m^-1 if not set or invalid

Example: `0.15` for sharper turns in urban environments

## Validation Rules

1. Curvature values must be non-negative
2. Curvature values must be in ascending order
3. Gain multipliers must be ≥ 1.0
4. Both arrays must have matching lengths
5. Curvature values must not exceed the maximum limit (default or custom)
6. MaxCurvatureGainMultiplier must be between 1.0 and 10.0