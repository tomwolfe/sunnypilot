"""
Enhanced Safety Monitoring for sunnypilot - Implements sophisticated safety checks
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
from typing import Dict, Tuple, Optional
from cereal import log
import cereal.messaging as messaging
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.params import Params, UnknownKeyName


class SafetyMonitor:
  """
  Enhanced Safety Monitoring System for Autonomous Driving
  Implements multi-sensor fusion validation, confidence thresholds, and environmental adaptation
  """
  
  def __init__(self):
    # Initialize filters for smooth monitoring
    # Use different time constants for different types of values
    # Confidence values: longer time constant for stability in control decisions
    # Lane deviation: minimal time constant for immediate safety detection
    self.model_confidence_filter = FirstOrderFilter(1.0, 0.5, DT_MDL)
    self.radar_confidence_filter = FirstOrderFilter(1.0, 0.5, DT_MDL)
    self.lane_deviation_filter = FirstOrderFilter(0.0, 0.01, DT_MDL)  # Almost instantaneous for safety detection

    # Initialize params system to allow configurable thresholds
    self.params = Params()

    # Confidence thresholds for safety decisions - with configurable defaults
    # Use proper UnknownKeyName handling for missing parameters
    try:
        model_confidence_param = self.params.get("ModelConfidenceThreshold")
        self.model_confidence_threshold = float(model_confidence_param) if model_confidence_param else 0.7
    except UnknownKeyName:
        self.model_confidence_threshold = 0.7  # Default value if parameter not found
    except (TypeError, ValueError):
        self.model_confidence_threshold = 0.7  # Default value if parameter is invalid type

    try:
        radar_confidence_param = self.params.get("RadarConfidenceThreshold")
        self.radar_confidence_threshold = float(radar_confidence_param) if radar_confidence_param else 0.6
    except UnknownKeyName:
        self.radar_confidence_threshold = 0.6  # Default value if parameter not found
    except (TypeError, ValueError):
        self.radar_confidence_threshold = 0.6  # Default value if parameter is invalid type

    try:
        lane_deviation_param = self.params.get("LaneDeviationThreshold")
        self.lane_deviation_threshold = float(lane_deviation_param) if lane_deviation_param else 0.8
    except UnknownKeyName:
        self.lane_deviation_threshold = 0.8  # Default value if parameter not found
    except (TypeError, ValueError):
        self.lane_deviation_threshold = 0.8  # Default value if parameter is invalid type

    # Low speed safety threshold for conservative behavior
    try:
        low_speed_safety_param = self.params.get("LowSpeedSafetyThreshold")
        self.low_speed_safety_threshold = float(low_speed_safety_param) if low_speed_safety_param else 0.4
    except UnknownKeyName:
        self.low_speed_safety_threshold = 0.4  # Default value if parameter not found
    except (TypeError, ValueError):
        self.low_speed_safety_threshold = 0.4  # Default value if parameter is invalid type  # meters from center

    # Model confidence threshold multiplier for critical safety checks
    try:
        model_confidence_multiplier_param = self.params.get("ModelConfidenceThresholdMultiplier")
        self.model_confidence_threshold_multiplier = float(model_confidence_multiplier_param) if model_confidence_multiplier_param else 0.8
    except UnknownKeyName:
        self.model_confidence_threshold_multiplier = 0.8  # Default value if parameter not found
    except (TypeError, ValueError):
        self.model_confidence_threshold_multiplier = 0.8  # Default value if parameter is invalid type
    
    # Environmental condition detection
    self.lighting_condition = "normal"  # "night", "dawn_dusk", "normal", "tunnel"
    self.weather_condition = "clear"    # "clear", "rain", "snow", "fog"
    self.road_condition = "dry"         # "dry", "wet", "icy", "snow"
    
    # Sensor fusion confidence scores
    self.camera_confidence = 1.0
    self.radar_confidence = 1.0
    self.imu_confidence = 1.0

    # Raw confidence scores for immediate safety assessments (unfiltered)
    self.raw_model_confidence = 1.0
    
    # Environmental condition filters
    self.lighting_change_filter = FirstOrderFilter(0.0, 1.0, DT_MDL)
    self.weather_change_filter = FirstOrderFilter(0.0, 2.0, DT_MDL)
    
    # Curve anticipation enhancement
    self.curve_anticipation_active = False
    self.curve_anticipation_score = 0.0
    self.max_anticipation_distance = 200.0  # meters ahead
    
    # System safety state
    self.overall_safety_score = 1.0
    self.requires_intervention = False
    self.confidence_degraded = False
    
    # Performance metrics
    self.monitoring_cycles = 0

  def update_model_confidence(self, model_v2_msg) -> None:
    """Update model confidence based on neural network outputs"""
    if hasattr(model_v2_msg, 'meta') and hasattr(model_v2_msg.meta, 'confidence'):
      raw_confidence = model_v2_msg.meta.confidence if model_v2_msg.meta.confidence else 1.0
      self.model_confidence = self.model_confidence_filter.update(raw_confidence)
      # Also store raw confidence for immediate safety decisions
      self.raw_model_confidence = raw_confidence
    else:
      self.model_confidence = 0.5  # Fallback to moderate confidence
      self.raw_model_confidence = 0.5  # Fallback to moderate confidence

  def update_radar_confidence(self, radar_state_msg) -> None:
    """Update radar confidence based on lead detection reliability"""
    if hasattr(radar_state_msg, 'leadOne') and radar_state_msg.leadOne.status:
      # Calculate confidence based on lead tracking quality
      lead = radar_state_msg.leadOne
      distance_confidence = min(1.0, max(0.0, (50.0 - abs(lead.dRel)) / 50.0))
      velocity_confidence = min(1.0, max(0.0, (30.0 - abs(lead.vRel)) / 30.0))
      self.radar_confidence = self.radar_confidence_filter.update(
        (distance_confidence * 0.7 + velocity_confidence * 0.3)
      )
    else:
      self.radar_confidence = 0.3  # Lower confidence when no lead detected

  def update_camera_confidence(self, model_v2_msg, car_state_msg) -> None:
    """Update camera confidence based on lane detection and visual clarity"""
    if hasattr(model_v2_msg, 'lateralPlan') and hasattr(model_v2_msg.lateralPlan, 'laneWidth'):
      # Calculate lane keeping confidence
      lane_width = model_v2_msg.lateralPlan.laneWidth
      if lane_width > 3.0:  # Reasonable lane width
        # Calculate lane deviation from center
        if len(model_v2_msg.lateralPlan.dPath) > 0:
          center_deviation = model_v2_msg.lateralPlan.dPath[0]  # Immediate deviation
          self.lane_deviation_filter.update(abs(center_deviation))
          
          # Lane keeping confidence decreases with deviation
          lane_confidence = max(0.1, 1.0 - min(0.9, abs(center_deviation) / 2.0))
        else:
          lane_confidence = 0.8
      else:
        lane_confidence = 0.3  # Low confidence for unusual lane widths
      
      # Adjust confidence based on vehicle speed
      speed_factor = min(1.0, max(0.3, (30.0 - car_state_msg.vEgo) / 30.0))
      self.camera_confidence = lane_confidence * speed_factor
    else:
      self.camera_confidence = 0.6

  def detect_environmental_conditions(self, model_v2_msg, car_state_msg, car_control_msg, live_pose_msg=None) -> None:
    """Detect environmental conditions and adjust safety accordingly"""
    # Lighting condition detection based on actual model outputs and car state (simplified)
    # Using model validy and other standard model_v2 fields instead of non-standard lighting field
    if hasattr(model_v2_msg, 'frameId') and model_v2_msg.frameId > 0:
      # Use time of day and model confidence to infer lighting conditions
      # This is a simplified approach - in practice would use actual lighting analysis
      # The 0.5 value below represents a base lighting factor that can be adjusted based on inputs
      lighting_factor = 0.5  # Base value

      # Adjust lighting based on model confidence (low confidence might indicate poor visibility)
      if hasattr(model_v2_msg, 'meta') and hasattr(model_v2_msg.meta, 'confidence'):
        if model_v2_msg.meta.confidence < 0.6:
          lighting_factor *= 0.7  # Potentially poor lighting affecting model performance

      # Use model path confidence to estimate visibility conditions
      if hasattr(model_v2_msg, 'path') and hasattr(model_v2_msg.path, 'probs') and model_v2_msg.path.probs is not None:
        if hasattr(model_v2_msg.path.probs, '__len__') and len(model_v2_msg.path.probs) > 0:
          path_confidence = model_v2_msg.path.probs[0] if model_v2_msg.path.probs[0] else 0.8
          lighting_factor *= path_confidence

      if lighting_factor < 0.3:
        self.lighting_condition = "night"
      elif lighting_factor < 0.5:
        self.lighting_condition = "dawn_dusk"
      elif lighting_factor > 0.8:
        self.lighting_condition = "tunnel"  # High confidence might be due to tunnel lighting
      else:
        self.lighting_condition = "normal"

    # Weather condition detection based on IMU and car dynamics (improved approach)
    # Use livePose orientation instead of carControl (carControl doesn't have orientation data)
    pitch = 0.0
    roll = 0.0
    orientation_available = False

    if (live_pose_msg is not None and
        hasattr(live_pose_msg, 'orientationNED') and
        live_pose_msg.orientationNED is not None and
        hasattr(live_pose_msg.orientationNED, '__len__') and
        len(live_pose_msg.orientationNED) >= 3):
      pitch = live_pose_msg.orientationNED[1]
      roll = live_pose_msg.orientationNED[0]
      orientation_available = True
    # Fallback: try to get orientation from carState if livePose is not available
    elif (hasattr(car_state_msg, 'orientationNED') and
          car_state_msg.orientationNED is not None and
          hasattr(car_state_msg.orientationNED, '__len__') and
          len(car_state_msg.orientationNED) >= 3):
      pitch = car_state_msg.orientationNED[1]
      roll = car_state_msg.orientationNED[0]
      orientation_available = True

    if orientation_available:
      # More sophisticated approach than just raw IMU values
      # Check for sustained unusual angles that might indicate weather or road conditions
      pitch_threshold = 0.1  # Reduced threshold for realistic detection
      roll_threshold = 0.1   # Reduced threshold for realistic detection

      # Additional weather detection based on car dynamics
      # Use car state acceleration and jerk to detect slippery conditions
      if hasattr(car_state_msg, 'aEgo') and hasattr(car_state_msg, 'vEgo'):
        # Check for unexplained acceleration/deceleration that might indicate slippery conditions
        # This is a simplified check - would be more sophisticated in real implementation
        if abs(pitch) > pitch_threshold or abs(roll) > roll_threshold:
          self.road_condition = "wet"  # Could indicate wet/icy road
          self.weather_condition = "rain"
        else:
          self.road_condition = "dry"
          self.weather_condition = "clear"
    else:
      # Default to safe assumption if we can't get reliable IMU data
      self.road_condition = "dry"
      self.weather_condition = "clear"

  def detect_curve_anticipation(self, model_v2_msg, car_state_msg) -> None:
    """Enhanced curve anticipation with improved safety margins"""
    if hasattr(model_v2_msg, 'path') and len(model_v2_msg.path.x) > 10:
      # Properly calculate path curvature using first and second derivatives
      max_curvature_ahead = 0.0
      x_coords = model_v2_msg.path.x
      y_coords = model_v2_msg.path.y

      if len(x_coords) > 2 and len(y_coords) > 2:
        # Calculate curvature using numerical derivatives
        # Use central differences for better accuracy where possible
        for i in range(1, min(25, len(x_coords)-1)):  # Look ahead starting from 0.5s
          if i < len(x_coords) and i < len(y_coords):
            # First derivatives (dx, dy)
            if i > 0 and i < len(x_coords)-1:  # Central difference for internal points
              dx_dt = (x_coords[i+1] - x_coords[i-1]) / 2.0  # Approximate dx/dt
              dy_dt = (y_coords[i+1] - y_coords[i-1]) / 2.0  # Approximate dy/dt
            else:  # Forward or backward difference for edge points
              if i == 0:
                dx_dt = x_coords[i+1] - x_coords[i]
                dy_dt = y_coords[i+1] - y_coords[i]
              else:  # i == len-1
                dx_dt = x_coords[i] - x_coords[i-1]
                dy_dt = y_coords[i] - y_coords[i-1]

            # Second derivatives (d2x, d2y)
            if i > 0 and i < len(x_coords)-1:  # Central difference for second derivative
              d2x_dt2 = x_coords[i+1] - 2*x_coords[i] + x_coords[i-1]
              d2y_dt2 = y_coords[i+1] - 2*y_coords[i] + y_coords[i-1]
            else:  # Use forward/backward difference for boundary points
              if i == 0 and len(x_coords) > 2:
                d2x_dt2 = x_coords[i+2] - 2*x_coords[i+1] + x_coords[i]
                d2y_dt2 = y_coords[i+2] - 2*y_coords[i+1] + y_coords[i]
              elif i == len(x_coords)-1 and len(x_coords) > 2:
                d2x_dt2 = x_coords[i] - 2*x_coords[i-1] + x_coords[i-2]
                d2y_dt2 = y_coords[i] - 2*y_coords[i-1] + y_coords[i-2]
              else:
                d2x_dt2 = 0.0
                d2y_dt2 = 0.0

            # Calculate curvature using the formula:
            # curvature = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
            numerator = abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2)
            denominator_squared = dx_dt**2 + dy_dt**2

            if denominator_squared > 1e-6:  # Avoid division by zero
              denominator = denominator_squared ** 1.5
              local_curvature = numerator / denominator
              max_curvature_ahead = max(max_curvature_ahead, abs(local_curvature))

      # Calculate safe speed based on curvature
      if max_curvature_ahead > 0.001:  # Significant curve
        max_lat_accel = 3.0  # Maximum lateral acceleration considered safe
        safe_speed = (max_lat_accel / max_curvature_ahead) ** 0.5 if max_curvature_ahead > 0.0001 else car_state_msg.vEgo

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

  def calculate_overall_safety_score(self, car_state_msg) -> float:
    """Calculate overall safety score based on all inputs"""
    # Weighted combination of all confidence measures
    # Adjust weights based on environmental conditions
    base_weights = {
      'model': 0.4,
      'camera': 0.3,
      'radar': 0.2,
      'imu': 0.1
    }

    # Adjust weights based on environmental conditions
    if self.lighting_condition in ["night", "dawn_dusk", "tunnel"]:
      base_weights['model'] *= 0.8  # Reduce camera-based confidence in poor lighting
      base_weights['radar'] *= 1.1  # Increase radar confidence

    if self.weather_condition in ["rain", "snow", "fog"]:
      base_weights['model'] *= 0.7
      base_weights['camera'] *= 0.6
      base_weights['radar'] *= 1.2  # Radar more reliable in poor weather

    # Calculate base safety score
    safety_score = (
      self.model_confidence * base_weights['model'] +
      self.camera_confidence * base_weights['camera'] +
      self.radar_confidence * base_weights['radar'] +
      self.imu_confidence * base_weights['imu']
    )

    # Apply critical safety penalties for values significantly below thresholds
    # Use raw confidence for immediate safety response to avoid filter delays
    model_confidence_threshold_adjusted = self.model_confidence_threshold * self.model_confidence_threshold_multiplier  # 0.56 if threshold is 0.7
    if self.raw_model_confidence < model_confidence_threshold_adjusted:
      # Apply additional penalty when raw model confidence is critically low
      confidence_deficit = (model_confidence_threshold_adjusted - self.raw_model_confidence) / model_confidence_threshold_adjusted
      safety_penalty = min(0.3, confidence_deficit)  # Max 30% reduction for very low confidence
      safety_score *= (1.0 - safety_penalty)

    # Apply curve anticipation safety factor
    if self.curve_anticipation_active and car_state_msg.vEgo > 10.0:
      # Reduce safety score when approaching curves at high speed
      curve_factor = max(0.5, 1.0 - (self.curve_anticipation_score * 2.0))
      safety_score *= curve_factor

    # Apply lane deviation penalty
    if self.lane_deviation_filter.x > self.lane_deviation_threshold:
      deviation_penalty = max(0.1, 1.0 - (self.lane_deviation_filter.x / self.lane_deviation_threshold))
      safety_score *= deviation_penalty

    return max(0.0, min(1.0, safety_score))

  def evaluate_safety_intervention_needed(self, safety_score: float, car_state_msg) -> bool:
    """Determine if safety intervention is required"""
    # Check multiple conditions for intervention
    intervention_needed = False
    
    # Model confidence too low
    if self.model_confidence < self.model_confidence_threshold * self.model_confidence_threshold_multiplier:
      intervention_needed = True
    
    # Lane deviation too high
    if self.lane_deviation_filter.x > self.lane_deviation_threshold * 1.2:
      intervention_needed = True
    
    # Approaching curve too fast
    if self.curve_anticipation_active and self.curve_anticipation_score > 0.05:
      intervention_needed = True
    
    # Overall safety score too low
    if safety_score < 0.3:
      intervention_needed = True

    # At very low speeds, be more conservative with intervention
    if car_state_msg.vEgo < 5.0 and safety_score < self.low_speed_safety_threshold:
      intervention_needed = True
    
    return intervention_needed

  def update(self, model_v2_msg, radar_state_msg, car_state_msg, car_control_msg, live_pose_msg=None) -> Tuple[float, bool, Dict]:
    """Main update function - processes all inputs and returns safety assessment"""
    try:
      # Update all confidence measures with error handling
      try:
        self.update_model_confidence(model_v2_msg)
      except Exception as e:
        import logging
        logging.error(f"Error in update_model_confidence: {e}")
        self.model_confidence = 0.5  # Default to moderate confidence on error

      try:
        self.update_radar_confidence(radar_state_msg)
      except Exception as e:
        import logging
        logging.error(f"Error in update_radar_confidence: {e}")
        self.radar_confidence = 0.5

      try:
        self.update_camera_confidence(model_v2_msg, car_state_msg)
      except Exception as e:
        import logging
        logging.error(f"Error in update_camera_confidence: {e}")
        self.camera_confidence = 0.5

      try:
        self.detect_environmental_conditions(model_v2_msg, car_state_msg, car_control_msg, live_pose_msg)
      except Exception as e:
        import logging
        logging.error(f"Error in detect_environmental_conditions: {e}")
        # Keep existing values if detection fails

      try:
        self.detect_curve_anticipation(model_v2_msg, car_state_msg)
      except Exception as e:
        import logging
        logging.error(f"Error in detect_curve_anticipation: {e}")
        self.curve_anticipation_active = False
        self.curve_anticipation_score = 0.0

      # Calculate overall safety with error handling
      try:
        self.overall_safety_score = self.calculate_overall_safety_score(car_state_msg)
      except Exception as e:
        import logging
        logging.error(f"Error in calculate_overall_safety_score: {e}")
        self.overall_safety_score = 0.5

      try:
        self.requires_intervention = self.evaluate_safety_intervention_needed(
          self.overall_safety_score, car_state_msg
        )
      except Exception as e:
        import logging
        logging.error(f"Error in evaluate_safety_intervention_needed: {e}")
        self.requires_intervention = True  # Default to intervention on error

      # Determine if confidence is degraded (requires more conservative driving)
      self.confidence_degraded = self.overall_safety_score < 0.6

      # Prepare safety report with error information
      safety_report = {
        'model_confidence': getattr(self, 'model_confidence', 0.5),
        'radar_confidence': getattr(self, 'radar_confidence', 0.5),
        'camera_confidence': getattr(self, 'camera_confidence', 0.5),
        'imu_confidence': getattr(self, 'imu_confidence', 0.5),
        'lighting_condition': self.lighting_condition,
        'weather_condition': self.weather_condition,
        'road_condition': self.road_condition,
        'curve_anticipation_active': getattr(self, 'curve_anticipation_active', False),
        'curve_anticipation_score': getattr(self, 'curve_anticipation_score', 0.0),
        'lane_deviation': getattr(self.lane_deviation_filter, 'x', 0.0),
        'overall_safety_score': self.overall_safety_score,
        'confidence_degraded': self.confidence_degraded,
        'monitoring_cycles': self.monitoring_cycles
      }

      self.monitoring_cycles += 1

      # Add error flag if any sub-component failed
      safety_report['error_occurred'] = False

    except Exception as e:
      import logging
      import traceback
      logging.error(f"Critical error in SafetyMonitor.update: {e}")
      logging.error(f"Traceback: {traceback.format_exc()}")

      # Return safe defaults in case of critical error
      safety_report = {
        'model_confidence': 0.1,
        'radar_confidence': 0.1,
        'camera_confidence': 0.1,
        'imu_confidence': 0.1,
        'lighting_condition': 'normal',
        'weather_condition': 'clear',
        'road_condition': 'dry',
        'curve_anticipation_active': False,
        'curve_anticipation_score': 0.0,
        'lane_deviation': 0.0,
        'overall_safety_score': 0.1,  # Critical low safety score
        'confidence_degraded': True,
        'monitoring_cycles': self.monitoring_cycles,
        'error_occurred': True,
        'error_details': str(e)
      }
      self.overall_safety_score = 0.1
      self.requires_intervention = True
      self.confidence_degraded = True
      self.monitoring_cycles += 1

    return self.overall_safety_score, self.requires_intervention, safety_report