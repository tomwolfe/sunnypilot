"""
Road Model Validator Module
Enhanced perception validation for sunnypilot autonomous driving system

This module provides advanced validation and correction for road model outputs
from the neural network to ensure safe and reliable perception.

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
This file is part of sunnypilot and is licensed under the MIT License.
"""

import numpy as np
from typing import Any
import math

from openpilot.common.swaglog import cloudlog


class RoadModelValidator:
    """Validates and corrects road model outputs from neural network."""

    def __init__(self):
        """Initialize the road model validator with thresholds and parameters."""
        # Lane line validation parameters
        self.max_lane_curvature = 0.2  # Maximum reasonable lane curvature (1/m)
        self.min_lane_distance = 0.5   # Minimum distance between lanes (m)
        self.max_lane_distance = 5.0   # Maximum reasonable lateral distance (m)

        # Path planning validation parameters
        self.max_path_curvature = 0.3  # Maximum reasonable path curvature (1/m)
        self.curvature_change_limit = 0.1  # Maximum allowed curvature change per meter

        # Lead vehicle validation parameters
        self.min_lead_distance = 1.0   # Minimum valid lead distance (m)
        self.max_lead_distance = 200.0 # Maximum valid lead distance (m)
        self.max_lead_velocity = 100.0 # Maximum valid lead velocity (m/s)

        # Confidence thresholds
        self.min_lane_confidence = 0.1 # Minimum confidence for lane detection

        # Storage for previous values to enable temporal consistency checks
        self.prev_lane_lines = None
        self.prev_lane_line_probs = None
        self.prev_desired_curvature = 0.0
        self.prev_valid = True

    def validate_lane_lines(self, lane_lines: np.ndarray, lane_line_probs: np.ndarray = None) -> tuple[np.ndarray, bool]:
        """
        Validate lane line detections for physical reasonableness.

        Args:
            lane_lines: Array of lane line polynomial coefficients [n_lanes, 5] (for degree 4 polynomial)
            lane_line_probs: Array of lane line probabilities [n_lanes]

        Returns:
            Tuple of (corrected_lane_lines, is_valid)
        """
        if lane_lines is None or len(lane_lines) == 0:
            cloudlog.debug("Lane lines validation: no data to validate")
            return lane_lines, False

        # Make a copy to modify
        corrected_lines = lane_lines.copy()
        is_valid = True

        # Validate polynomial coefficients for physical reasonableness
        for i, line_coeffs in enumerate(corrected_lines):
            if len(line_coeffs) < 3:  # Need at least quadratic for road modeling
                cloudlog.warning(f"Lane line {i} has insufficient coefficients")
                is_valid = False
                continue

            # Check that higher-order coefficients are reasonable
            # For road polynomials, higher-order terms should be small
            if len(line_coeffs) > 2 and abs(line_coeffs[2]) > 0.01:  # x^2 coefficient
                cloudlog.warning(f"Excessive x^2 coefficient for lane line {i}: {line_coeffs[2]}")
                corrected_lines[i][2] = np.clip(line_coeffs[2], -0.01, 0.01)
                is_valid = False

            if len(line_coeffs) > 3 and abs(line_coeffs[3]) > 0.001:  # x^3 coefficient
                cloudlog.warning(f"Excessive x^3 coefficient for lane line {i}: {line_coeffs[3]}")
                corrected_lines[i][3] = np.clip(line_coeffs[3], -0.001, 0.001)
                is_valid = False

            # Check that lane line is not too curved at typical viewing distances
            # Evaluate curvature at 5m, 15m, 25m ahead
            for distance in [5.0, 15.0, 25.0]:
                # Calculate curvature = |f''| / (1 + (f')^2)^(3/2)
                # where f(x) = c0 + c1*x + c2*x^2 + c3*x^3 + c4*x^4
                # f'(x) = c1 + 2*c2*x + 3*c3*x^2 + 4*c4*x^3
                # f''(x) = 2*c2 + 6*c3*x + 12*c4*x^2
                if len(line_coeffs) >= 3:
                    first_deriv = line_coeffs[1] + 2*line_coeffs[2]*distance + (3*line_coeffs[3]*distance**2 if len(line_coeffs) > 3 else 0)
                    second_order = (6*line_coeffs[3]*distance if len(line_coeffs) > 3 else 0)
                    third_order = (12*line_coeffs[4]*distance**2 if len(line_coeffs) > 4 else 0)
                    second_deriv = 2*line_coeffs[2] + second_order + third_order

                    curvature = abs(second_deriv) / (1 + first_deriv**2)**1.5 if 1 + first_deriv**2 > 0.1 else abs(second_deriv)

                    if abs(curvature) > self.max_lane_curvature:
                        cloudlog.warning(f"Lane line {i} has excessive curvature {curvature:.3f} at {distance}m: {line_coeffs}")
                        # Apply correction by reducing higher-order coefficients
                        corrected_lines[i][2] *= 0.7  # Reduce x^2 coefficient
                        if len(corrected_lines[i]) > 3:
                            corrected_lines[i][3] *= 0.5  # Reduce x^3 coefficient
                        is_valid = False

        # Check lane line distances (between lines should be reasonable)
        if len(corrected_lines) >= 2:
            # Calculate approximate distance between first two lane lines at 10m ahead
            line_0_at_10m = np.polyval(np.flip(corrected_lines[0]), 10.0)
            line_1_at_10m = np.polyval(np.flip(corrected_lines[1]), 10.0)
            distance_between_lines = abs(line_0_at_10m - line_1_at_10m)

            if distance_between_lines < self.min_lane_distance or distance_between_lines > self.max_lane_distance:
                cloudlog.warning(f"Lane line distance not reasonable: {distance_between_lines:.2f}m")
                is_valid = False

        # Enhanced temporal consistency check with previous frame
        if self.prev_lane_lines is not None and len(self.prev_lane_lines) > 0:
            # Compare current detection with previous to detect anomalies
            if len(corrected_lines) != len(self.prev_lane_lines):
                cloudlog.debug("Lane detection count changed significantly from previous frame")

            # Check for excessive deviation from previous frame (temporal consistency)
            for i in range(min(len(corrected_lines), len(self.prev_lane_lines))):
                line_diff = np.abs(corrected_lines[i] - self.prev_lane_lines[i])
                if np.any(line_diff > 0.5):  # Significant deviation threshold
                    cloudlog.debug(f"Lane line {i} deviated significantly from previous frame: {line_diff}")
                    is_valid = False

        # If lane probabilities are provided, validate against thresholds
        if lane_line_probs is not None and len(lane_line_probs) == len(corrected_lines):
            for i, prob in enumerate(lane_line_probs):
                if prob < self.min_lane_confidence:
                    cloudlog.debug(f"Lane line {i} confidence too low: {prob:.3f}")
                    # Still consider valid if physically reasonable, but log

        return corrected_lines, is_valid

    def validate_path_planning(self, position: np.ndarray, velocity: np.ndarray = None) -> tuple[np.ndarray, bool]:
        """
        Validate planned path for safety and physical reasonableness.

        Args:
            position: Array of planned positions [timesteps, 2] (x, y) or [timesteps, 3] (x, y, z)
            velocity: Array of planned velocities [timesteps, 3] (x, y, z) if available

        Returns:
            Tuple of (corrected_position, is_valid)
        """
        if position is None or len(position) == 0:
            return position, False

        corrected_pos = position.copy()
        is_valid = True

        # Calculate path curvature
        if len(position) >= 2:
            # Calculate curvature along path
            for i in range(1, len(position)-1):
                # Use three consecutive points to calculate approximate curvature
                p_prev = position[i-1]
                p_curr = position[i]
                p_next = position[i+1]

                # Vector from prev to curr
                v1 = p_curr[:2] - p_prev[:2]  # Only use x, y for curvature calculation
                # Vector from curr to next
                v2 = p_next[:2] - p_curr[:2]

                # Calculate angle between vectors
                if np.linalg.norm(v1) > 0.1 and np.linalg.norm(v2) > 0.1:
                    cos_angle = np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0)
                    angle = math.acos(cos_angle)

                    # Approximate curvature as angle change per distance
                    dist = np.linalg.norm(v1)  # Approximate path length segment
                    if dist > 0:
                        curvature = angle / dist
                        if abs(curvature) > self.max_path_curvature:
                            cloudlog.warning(f"Excessive path curvature {curvature:.3f} at index {i}")
                            is_valid = False

                            # Try to smooth this point
                            corrected_pos[i] = (corrected_pos[i-1] + corrected_pos[i+1]) / 2.0

        return corrected_pos, is_valid

    def validate_lead_objects(self, leads_v3: list) -> tuple[list, bool]:
        """
        Validate lead vehicle detections.

        Args:
            leads_v3: List of lead objects from model output

        Returns:
            Tuple of (corrected_leads, is_valid)
        """
        if not leads_v3:
            return leads_v3, True  # Empty is valid

        corrected_leads = []
        is_valid = True

        for i, lead in enumerate(leads_v3):
            if not hasattr(lead, 'dRel') or not hasattr(lead, 'vRel'):
                cloudlog.warning(f"Lead {i} missing required attributes")
                continue

            # Validate distance
            if lead.dRel < self.min_lead_distance or lead.dRel > self.max_lead_distance:
                cloudlog.warning(f"Lead {i} distance unreasonable: {lead.dRel}m")
                is_valid = False
                continue  # Skip this lead

            # Validate velocity
            if abs(lead.vRel) > self.max_lead_velocity:
                cloudlog.warning(f"Lead {i} velocity unreasonable: {lead.vRel}m/s")
                is_valid = False
                continue  # Skip this lead

            # Additional validation: if distance is very close but velocity suggests no collision risk
            if lead.dRel < 10.0 and lead.vRel > 5.0:  # Close but lead is moving away rapidly
                # This could be a detection error
                cloudlog.warning(f"Lead {i} very close but moving away rapidly: {lead.dRel}m, {lead.vRel}m/s")
                is_valid = False

            # Validate tOffset (time offset to lead)
            if hasattr(lead, 'tOffset') and abs(lead.tOffset) > 5.0:
                cloudlog.warning(f"Lead {i} tOffset unreasonable: {lead.tOffset}s")
                is_valid = False

            # All validations passed, keep this lead
            corrected_leads.append(lead)

        return corrected_leads, is_valid

    def validate_model_output(self, model_output: dict[str, Any], v_ego: float) -> tuple[dict[str, Any], bool]:
        """
        Comprehensive validation of model outputs.

        Args:
            model_output: Dictionary of model outputs from modelV2
            v_ego: Current vehicle speed

        Returns:
            Tuple of (corrected_model_output, is_valid)
        """
        is_valid = True

        # Validate lane lines
        if 'laneLines' in model_output:
            lane_lines, lane_valid = self.validate_lane_lines(model_output['laneLines'])
            model_output['laneLines'] = lane_lines
            if not lane_valid:
                cloudlog.debug("Model output: lane lines validation failed")
                is_valid = False

        # Validate path planning
        if 'position' in model_output:
            corrected_pos, path_valid = self.validate_path_planning(model_output['position'])
            model_output['position'] = corrected_pos
            if not path_valid:
                cloudlog.debug("Model output: path planning validation failed")
                is_valid = False

        # Validate lead objects
        if 'leadsV3' in model_output:
            corrected_leads, leads_valid = self.validate_lead_objects(model_output['leadsV3'])
            model_output['leadsV3'] = corrected_leads
            if not leads_valid:
                cloudlog.debug("Model output: lead objects validation failed")
                is_valid = False

        # Validate desired curvature from action
        if 'action' in model_output and hasattr(model_output['action'], 'desiredCurvature'):
            desired_curvature = model_output['action'].desiredCurvature
            # Check for extreme curvature that's not physically possible at speed
            if abs(desired_curvature) > self.max_path_curvature and v_ego > 5.0:
                max_safe_curvature = 3.0 / (v_ego ** 2) if v_ego > 0.1 else self.max_path_curvature
                if abs(desired_curvature) > max_safe_curvature:
                    cloudlog.warning(f"Desired curvature {desired_curvature} too high for speed {v_ego}m/s (max safe: {max_safe_curvature})")
                    model_output['action'].desiredCurvature = max(-max_safe_curvature, min(max_safe_curvature, desired_curvature))
                    is_valid = False
            # Check for physically impossible curvatures
            if abs(desired_curvature) > 1.0:  # Radius less than 1m
                cloudlog.error(f"Physically impossible desired curvature: {desired_curvature}")
                model_output['action'].desiredCurvature = max(-0.5, min(0.5, desired_curvature))
                is_valid = False

        # Add validation flag to output
        if 'meta' not in model_output:
            model_output['meta'] = {}

        # Preserve original validation flag and add our result
        original_validation = model_output['meta'].get('validation_applied', False)
        model_output['meta']['validation_applied'] = original_validation or not is_valid
        model_output['meta']['road_model_validation'] = not is_valid

        return model_output, is_valid

    def update_temporal_consistency(self, model_output: dict[str, Any], v_ego: float):
        """Update temporal consistency checks using previous frame data."""
        # Store current values for next frame comparison
        if 'laneLines' in model_output:
            self.prev_lane_lines = model_output['laneLines'].copy() if model_output['laneLines'] is not None else None

        if 'action' in model_output and hasattr(model_output['action'], 'desiredCurvature'):
            self.prev_desired_curvature = model_output['action'].desiredCurvature


# Global validator instance for reuse
road_model_validator = RoadModelValidator()
