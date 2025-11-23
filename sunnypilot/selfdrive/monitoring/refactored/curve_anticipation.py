"""
Curve Anticipation for sunnypilot - Separated module for curve detection and anticipation

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import logging


class CurveAnticipation:
    """
    Curve anticipation and safe speed calculation
    Separated from main SafetyMonitor to reduce complexity
    """
    def __init__(self, max_lat_accel: float = 2.0, max_anticipation_distance: float = 200.0):
        self.max_lat_accel = max_lat_accel
        self.max_anticipation_distance = max_anticipation_distance
        self.curve_anticipation_active = False
        self.curve_anticipation_score = 0.0

    def detect_and_anticipate_curve(self, model_v2_msg, car_state_msg) -> tuple:
        """Detect curves ahead and calculate anticipation metrics"""
        if hasattr(model_v2_msg, 'path') and len(model_v2_msg.path.x) > 10:
            # Properly calculate path curvature using first and second derivatives
            max_curvature_ahead = 0.0
            x_coords = model_v2_msg.path.x
            y_coords = model_v2_msg.path.y
            z_coords = getattr(model_v2_msg.path, 'z', [0.0] * len(x_coords))  # Use default flat path if z not available

            if len(x_coords) > 2 and len(y_coords) > 2:
                # Convert lists to numpy arrays for efficient computation
                x_coords_np = np.array(x_coords)
                y_coords_np = np.array(y_coords)
                z_coords_np = np.array(z_coords) if len(z_coords) >= len(x_coords) else np.array([0.0] * len(x_coords))

                # Assuming path points are equally spaced (0.5m)
                path_interval = 0.5 # Typically 0.5 meters between path points

                # Add proper path length validation
                min_points_needed = int(self.max_anticipation_distance / path_interval)
                current_max_anticipation_distance = self.max_anticipation_distance
                if len(x_coords_np) < min_points_needed:
                    logging.warning(f"Path is shorter than expected for {self.max_anticipation_distance}m anticipation. Actual: {len(x_coords_np)} points, Expected: {min_points_needed} points. Using available path length.")
                    # Fall back to shorter distance for calculation
                    current_max_anticipation_distance = len(x_coords_np) * path_interval

                # Calculate first derivatives (dx/ds, dy/ds) and second derivatives
                dx_ds = np.gradient(x_coords_np, path_interval)
                dy_ds = np.gradient(y_coords_np, path_interval)
                d2x_ds2 = np.gradient(dx_ds, path_interval)
                d2y_ds2 = np.gradient(dy_ds, path_interval)

                # Calculate dz/ds (slope along the path)
                dz_ds = np.gradient(z_coords_np, path_interval) if len(z_coords_np) == len(x_coords_np) else np.array([0.0] * len(x_coords_np))
                
                # Calculate curvature using the vectorized formula: curvature = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
                numerator = np.abs(dx_ds * d2y_ds2 - dy_ds * d2x_ds2)
                denominator_squared = dx_ds**2 + dy_ds**2

                # Avoid division by zero: set curvature to 0 where denominator is too small
                valid_indices = denominator_squared > 1e-6

                local_curvatures = np.zeros_like(numerator)
                local_curvatures[valid_indices] = numerator[valid_indices] / (denominator_squared[valid_indices]**1.5)

                effective_path_length = min(len(x_coords_np), int(current_max_anticipation_distance / path_interval))

                if effective_path_length > 0:
                    max_curvature_ahead = np.max(np.abs(local_curvatures[:effective_path_length]))
                    # Get the grade at the point of max curvature for simplicity
                    max_curve_idx = np.argmax(np.abs(local_curvatures[:effective_path_length]))

                    # Simple approximation of grade at max curvature point
                    # Avoid division by zero for small dx_ds and dy_ds (flat path)
                    if np.linalg.norm([dx_ds[max_curve_idx], dy_ds[max_curve_idx]]) > 1e-6:
                        grade_at_max_curvature = dz_ds[max_curve_idx] / np.linalg.norm([dx_ds[max_curve_idx], dy_ds[max_curve_idx]])
                    else:
                        grade_at_max_curvature = 0.0
                else:
                    max_curvature_ahead = 0.0
                    grade_at_max_curvature = 0.0

                # Calculate safe speed based on curvature and now, road grade
                if max_curvature_ahead > 0.001:  # Significant curve
                    # Adjust max_lat_accel based on road grade for safety
                    # On a downhill grade, the effective lateral acceleration available might be reduced.
                    # On an uphill grade, it might be increased.
                    # A conservative approach for now: reduce safe speed on downhill grades.
                    grade_factor = 1.0
                    if grade_at_max_curvature < 0:  # Downhill
                        # Reduce safe speed for downhill curves. For example, a 5% grade (approx 0.05 rad)
                        # might reduce the effective lateral acceleration by a small percentage.
                        # This is a placeholder and needs empirical tuning.
                        grade_factor = max(0.8, 1.0 + grade_at_max_curvature * 5.0) # Reduce by up to 20% for steep downhill
                    elif grade_at_max_curvature > 0: # Uphill
                        # For uphill, we might slightly increase the safe speed, or keep it neutral for conservatism.
                        # For now, let's keep it neutral to prioritize safety.
                        grade_factor = 1.0

                    effective_max_lat_accel = self.max_lat_accel * grade_factor
                    safe_speed = (effective_max_lat_accel / max_curvature_ahead) ** 0.5 if max_curvature_ahead > 0.0001 else float(car_state_msg.vEgo)

                    # Calculate anticipation score based on speed vs safe speed
                    if car_state_msg.vEgo > 5.0:  # Only for meaningful speeds
                        speed_ratio = min(1.0, max(0.0, safe_speed / car_state_msg.vEgo))
                        self.curve_anticipation_score = max_curvature_ahead * (1.0 - speed_ratio)
                        self.curve_anticipation_active = speed_ratio < 0.9  # 10% margin
                    else:
                        self.curve_anticipation_score = 0.0
                        self.curve_anticipation_active = False
                else:
                    self.curve_anticipation_score = 0.0
                    self.curve_anticipation_active = False
            else:
                self.curve_anticipation_score = 0.0
                self.curve_anticipation_active = False
        else:
            self.curve_anticipation_score = 0.0
            self.curve_anticipation_active = False

        return self.curve_anticipation_active, self.curve_anticipation_score