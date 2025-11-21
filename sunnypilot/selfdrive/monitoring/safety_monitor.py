"""
Enhanced Safety Monitoring for sunnypilot - Implements sophisticated safety checks
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import logging
from typing import Dict, Tuple, Optional
import time # Import time for performance measurements
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
    # Justification: This threshold is set to a relatively high value (0.7) to ensure that the system operates only when the model's prediction
    # is sufficiently reliable. Lower values could lead to unsafe maneuvers. This value is subject to empirical tuning.
    try:
        model_confidence_param = self.params.get("ModelConfidenceThreshold")
        self.model_confidence_threshold = float(model_confidence_param) if model_confidence_param else 0.7
    except UnknownKeyName:
        self.model_confidence_threshold = 0.7  # Default value if parameter not found
    except (TypeError, ValueError):
        self.model_confidence_threshold = 0.7  # Default value if parameter is invalid type

    # Justification: This threshold (0.6) balances responsiveness to radar data with robustness against noise or false positives.
    # It allows for some uncertainty but demands a reasonable level of confidence from the radar system. Subject to empirical tuning.
    try:
        radar_confidence_param = self.params.get("RadarConfidenceThreshold")
        self.radar_confidence_threshold = float(radar_confidence_param) if radar_confidence_param else 0.6
    except UnknownKeyName:
        self.radar_confidence_threshold = 0.6  # Default value if parameter not found
    except (TypeError, ValueError):
        self.radar_confidence_threshold = 0.6  # Default value if parameter is invalid type

    # Justification: This threshold (0.8m) defines the maximum allowable deviation from the lane center before a safety concern is raised.
    # It's a balance between comfort and safety, preventing excessive weaving while allowing for normal driving adjustments.
    # This value needs to be tuned considering vehicle dynamics and typical lane widths.
    try:
        lane_deviation_param = self.params.get("LaneDeviationThreshold")
        self.lane_deviation_threshold = float(lane_deviation_param) if lane_deviation_param else 0.8
    except UnknownKeyName:
        self.lane_deviation_threshold = 0.8  # Default value if parameter not found
    except (TypeError, ValueError):
        self.lane_deviation_threshold = 0.8  # Default value if parameter is invalid type

    # Low speed safety threshold for conservative behavior
    # Justification: At low speeds (below ~20 km/h), control can be more sensitive, and immediate intervention might be critical.
    # A higher threshold (0.4) at low speeds prioritizes safety over smoothness in tight maneuvers.
    try:
        low_speed_safety_param = self.params.get("LowSpeedSafetyThreshold")
        self.low_speed_safety_threshold = float(low_speed_safety_param) if low_speed_safety_param else 0.4
    except UnknownKeyName:
        self.low_speed_safety_threshold = 0.4  # Default value if parameter not found
    except (TypeError, ValueError):
        self.low_speed_safety_threshold = 0.4  # Default value if parameter is invalid type  # meters from center

    # Model confidence threshold multiplier for critical safety checks
    # Justification: This multiplier (0.8) reduces the effective model confidence threshold during critical safety checks,
    # making the system more sensitive to low confidence situations. This creates a buffer for intervention.
    try:
        model_confidence_multiplier_param = self.params.get("ModelConfidenceThresholdMultiplier")
        self.model_confidence_threshold_multiplier = float(model_confidence_multiplier_param) if model_confidence_multiplier_param else 0.8
        # Add validation for multiplier to ensure it is within a valid range [0.5, 1.0]
        if not (0.5 <= self.model_confidence_threshold_multiplier <= 1.0):
            logging.warning(f"ModelConfidenceThresholdMultiplier {self.model_confidence_threshold_multiplier} out of valid range [0.5, 1.0]. Using default 0.8")
            self.model_confidence_threshold_multiplier = 0.8
    except UnknownKeyName:
        self.model_confidence_threshold_multiplier = 0.8  # Default value if parameter not found
    except (TypeError, ValueError):
        self.model_confidence_threshold_multiplier = 0.8  # Default value if parameter is invalid type
    
    # Curve anticipation parameters
    # Justification: This value (2.0 m/s^2) represents a conservative maximum lateral acceleration the vehicle is allowed to experience.
    # It's a key parameter for calculating safe speeds around curves. This value should be tuned based on vehicle dynamics and comfort.
    try:
        max_lat_accel_param = self.params.get("MaxLateralAcceleration")
        self.max_lat_accel = float(max_lat_accel_param) if max_lat_accel_param else 2.0
    except UnknownKeyName:
        self.max_lat_accel = 2.0
    except (TypeError, ValueError):
        self.max_lat_accel = 2.0

    # Justification: This threshold (0.5) determines what level of curvature is considered significant enough to trigger curve anticipation logic.
    # A higher value would mean only sharper curves are detected, while a lower value would trigger for gentler curves.
    # This needs to be tuned to balance between false positives and missing critical curves.
    try:
        curve_threshold_param = self.params.get("CurveDetectionThreshold")
        self.curve_detection_threshold = float(curve_threshold_param) if curve_threshold_param else 0.5
    except UnknownKeyName:
        self.curve_detection_threshold = 0.5
    except (TypeError, ValueError):
        self.curve_detection_threshold = 0.5

    # Radar confidence parameters
    # Justification: This parameter (150.0m) defines the maximum range at which radar detections contribute to confidence.
    # Beyond this distance, the reliability of radar measurements for immediate safety assessment may diminish,
    # or the relevance to the ego vehicle's immediate safety context decreases.
    try:
        max_radar_distance_param = self.params.get("MaxRadarDistanceForConfidence")
        self.max_radar_distance_for_confidence = float(max_radar_distance_param) if max_radar_distance_param else 150.0
    except UnknownKeyName:
        self.max_radar_distance_for_confidence = 150.0
    except (TypeError, ValueError):
        self.max_radar_distance_for_confidence = 150.0

    # Justification: This scale (30.0) is used to calculate confidence based on the relative velocity of lead vehicles.
    # A larger scale makes the confidence decay slower with increasing relative velocity, implying that even higher relative velocities
    # can be considered with some confidence. This value needs careful empirical tuning to match real-world radar performance and
    # safety requirements across various vehicle types and driving scenarios.
    try:
        velocity_confidence_scale_param = self.params.get("VelocityConfidenceScale")
        self.velocity_confidence_scale = float(velocity_confidence_scale_param) if velocity_confidence_scale_param else 30.0
    except UnknownKeyName:
        self.velocity_confidence_scale = 30.0
    except (TypeError, ValueError):
        self.velocity_confidence_scale = 30.0

    # Justification: This factor (1.0) determines how much the `max_radar_distance_for_confidence` scales with vehicle speed.
    # A higher factor means the system expects to track objects further away at higher speeds.
    # This value needs empirical tuning to reflect safe braking distances and radar capabilities at various speeds.
    try:
        speed_scaling_factor_param = self.params.get("SpeedScalingFactorForRadarDistance")
        self.speed_scaling_factor_for_radar_distance = float(speed_scaling_factor_param) if speed_scaling_factor_param else 1.0
    except UnknownKeyName:
        self.speed_scaling_factor_for_radar_distance = 1.0
    except (TypeError, ValueError):
        self.speed_scaling_factor_for_radar_distance = 1.0

    # Radar confidence parameters for acceleration
    # Justification: This threshold (5.0 m/s^2) determines the maximum acceleration difference considered 'safe' for radar confidence.
    # It assumes that relative accelerations beyond this value significantly reduce the reliability or safety relevance of lead vehicle data.
    # This value is critical and should be tuned based on vehicle dynamics and specific radar performance characteristics.
    try:
        acceleration_confidence_threshold_param = self.params.get("AccelerationConfidenceThreshold")
        self.acceleration_confidence_threshold = float(acceleration_confidence_threshold_param) if acceleration_confidence_threshold_param else 5.0
    except UnknownKeyName:
        self.acceleration_confidence_threshold = 5.0
    except (TypeError, ValueError):
        self.acceleration_confidence_threshold = 5.0

    # Environmental condition detection
    self.lighting_condition = "night"  # "night", "dawn_dusk", "normal", "tunnel"
    self.weather_condition = "unknown"    # "clear", "rain", "snow", "fog", "unknown"
    self.road_condition = "unknown"         # "dry", "wet", "icy", "snow", "unknown"
    self.in_tunnel = True # NEW: Tunnel detection state
    
    # Sensor fusion confidence scores
    self.camera_confidence = 1.0
    self.radar_confidence = 1.0
    self.imu_confidence = 1.0

    # Raw confidence scores for immediate safety assessments (unfiltered)
    self.raw_model_confidence = 1.0
    
    # Sensor health flags - True if healthy, False if failed or degraded
    self.model_healthy = True
    self.radar_healthy = True
    self.camera_healthy = True
    self.imu_healthy = True # For livePose data used in environmental detection
    self.driver_monitoring_healthy = True # New: for driver monitoring health

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

    # Sensor staleness threshold - if data is older than this, it's considered stale
    # Justification: 0.5 seconds is a reasonable threshold for real-time autonomous driving systems.
    # Older data significantly reduces the reliability and safety of decisions.
    try:
        staleness_threshold_param = self.params.get("SensorStalenessThreshold")
        self.STALENESS_THRESHOLD_SECONDS = float(staleness_threshold_param) if staleness_threshold_param else 0.5
    except (UnknownKeyName, ValueError):
        self.STALENESS_THRESHOLD_SECONDS = 0.5 # Default to 0.5 seconds
    
    # Store last update times for staleness checks
    self.last_model_time = 0
    self.last_radar_time = 0
    self.last_camera_time = 0
    self.last_imu_time = 0

  def update_model_confidence(self, model_v2_msg) -> None:
    """Update model confidence based on neural network outputs"""
    if hasattr(model_v2_msg, 'meta') and hasattr(model_v2_msg.meta, 'confidence'):
      raw_confidence = model_v2_msg.meta.confidence if model_v2_msg.meta.confidence else 1.0
      self.model_confidence = self.model_confidence_filter.update(raw_confidence)
      # Also store raw confidence for immediate safety decisions
      self.raw_model_confidence = raw_confidence
    else:
      logging.warning("modelV2 data not available. Defaulting model confidence to a conservative 0.1.")
      self.model_confidence = 0.1  # Fallback to conservative confidence
      self.raw_model_confidence = 0.1  # Fallback to conservative confidence

  def update_radar_confidence(self, radar_state_msg, car_state_msg) -> None:
    """Update radar confidence based on lead detection reliability"""
    if hasattr(radar_state_msg, 'leadOne') and radar_state_msg.leadOne.status:
      # Calculate confidence based on lead tracking quality with a continuous decay
      lead = radar_state_msg.leadOne
      # Use configurable maximum radar distance, dynamically adjusted by vehicle speed
      max_radar_distance_for_confidence_scaled = self.max_radar_distance_for_confidence + \
                                                  (car_state_msg.vEgo * self.speed_scaling_factor_for_radar_distance)
      distance_confidence = max(0.0, 1.0 - (abs(lead.dRel) / max_radar_distance_for_confidence_scaled))
      # Use configurable velocity confidence scale
      velocity_confidence = min(1.0, max(0.0, (self.velocity_confidence_scale - abs(lead.vRel)) / self.velocity_confidence_scale))
      # Add acceleration-based confidence component
      # Assuming lead.aRel exists and is populated in radar_state_msg.leadOne
      # Add acceleration-based confidence component
      # The critical review suggests a fallback of 0.7 if aRel is not available.
      acceleration_confidence = 0.7 # Fallback to a moderate confidence as suggested by critical review if aRel is not available or None.
      if hasattr(lead, 'aRel') and lead.aRel is not None:
        acceleration_confidence = min(1.0, max(0.0, (self.acceleration_confidence_threshold - abs(lead.aRel)) / self.acceleration_confidence_threshold))
      else:
        logging.warning("aRel not available for radar acceleration confidence. Defaulting to a moderate 0.7.")
      self.radar_confidence = self.radar_confidence_filter.update(
        (distance_confidence * 0.5 + velocity_confidence * 0.3 + acceleration_confidence * 0.2)
      )
    else:
      self.radar_confidence = 0.3  # Lower confidence when no lead detected

  def update_camera_confidence(self, model_v2_msg, car_state_msg) -> None:
    """Update camera confidence based on lane detection, visual clarity, and basic car state plausibility"""
    # Plausibility check for vEgo
    if not (0.0 <= car_state_msg.vEgo <= 45.0): # Assuming plausible speed range 0-162 km/h (0-45 m/s)
      logging.warning(f"CarState vEgo ({car_state_msg.vEgo:.2f} m/s) out of plausible range. Reducing camera confidence.")
      self.camera_confidence = 0.1
      self.camera_healthy = False
      return # Skip further processing if vEgo is implausible
    
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
    # Lighting condition detection - use a more sophisticated approach based on actual lighting data
    # Instead of just using model confidence, look for signs of tunnel lighting based on other indicators

    # TODO: Enhance lighting detection by integrating camera luminance measurements from modelV2
    # or dedicated lighting sensors.
    # Current limitation: Model confidence from modelV2 does not provide direct luminance data.
    # Without dedicated sensor inputs or advanced camera-based luminance analysis,
    # we default to 'normal' and log a warning.
    # Future integration should consider:
    # 1. Camera-based luminance estimation (e.g., from image statistics or model outputs).
    # 2. Dedicated ambient light sensors.
    logging.warning("Luminance data for lighting condition detection is not available. Defaulting to 'unknown' lighting condition for conservative safety.")
    self.lighting_condition = "unknown" # Default to 'unknown' for conservative safety when luminance data is not available.
    # TODO: Implement robust tunnel detection.
    # Tunnel detection requires specific visual cues (consistent lane markings, absence of sky, specific lighting patterns).
    # This cannot be reliably inferred without proper camera-based analysis or dedicated sensors.
    # Potential approaches for future integration:
    # 1. Analyze 'modelV2.laneLines' for consistent, strong lane detections without 'roadEdges' or 'sky' probabilities.
    # 2. Integrate with lighting condition estimation (once available) for sudden drops in luminance.
    # 3. Utilize GPS data if available for known tunnel locations.
    # Tunnel detection is not yet reliably implemented.
    # Tunnel detection requires specific visual cues (consistent lane markings, absence of sky, specific lighting patterns)
    # or dedicated sensors, which are not currently available.
    # Potential approaches for future integration:
    # 1. Analyze 'modelV2.laneLines' for consistent, strong lane detections without 'roadEdges' or 'sky' probabilities.
    # 2. Integrate with lighting condition estimation (once available) for sudden drops in luminance.
    # 3. Utilize GPS data if available for known tunnel locations.
    self.in_tunnel = False # Explicitly set to False as tunnel detection is not yet functional.

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

    if orientation_available:
      # Robust weather and road condition detection using multiple sensor inputs is not yet implemented.
      # Current limitation: Relies primarily on IMU data (via live_pose_msg orientation) which is a proxy.
      # To ensure conservative safety, defaulting to "unknown" for now and logging a warning.
      logging.warning("Comprehensive multi-sensor fusion for weather/road condition detection is not yet implemented. Defaulting to 'unknown'.")
      self.road_condition = "unknown"
      self.weather_condition = "unknown"
      self.imu_confidence = 1.0 # IMU data available, so confidence is high
    else:
      # If IMU data is not available, log a warning and maintain "unknown" state.
      logging.warning("IMU data for weather/road condition detection is not available. Maintaining 'unknown' environmental conditions.")
      self.imu_confidence = 0.1 # Set IMU confidence to a low value due to missing data.

  def detect_curve_anticipation(self, model_v2_msg, car_state_msg) -> None:
    """Enhanced curve anticipation with improved safety margins"""
    if hasattr(model_v2_msg, 'path') and len(model_v2_msg.path.x) > 10:
      # Properly calculate path curvature using first and second derivatives
      max_curvature_ahead = 0.0
      x_coords = model_v2_msg.path.x
      y_coords = model_v2_msg.path.y
      z_coords = model_v2_msg.path.z # Assuming z-coordinates are available similarly

      if len(x_coords) > 2 and len(y_coords) > 2 and len(z_coords) > 2:
        # Convert lists to numpy arrays for efficient computation
        x_coords_np = np.array(x_coords)
        y_coords_np = np.array(y_coords)
        z_coords_np = np.array(z_coords)

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
        dz_ds = np.gradient(z_coords_np, path_interval)
        # Approximate road grade (angle in radians)
        # Assuming ds is roughly constant and small enough for tan(angle) approx angle
        # grade_angle = np.arctan(dz_ds)
        # For a simplified approach, use a smoothed version of dz_ds or an average over a segment
        # Here, we will just use the dz_ds directly as a proxy for grade.

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
            # This is a simplification; a more robust solution would consider grade over the entire curve
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
        # On a downhill grade, the effective lateral friction available might be reduced.
        # On an uphill grade, it might be increased.
        # A conservative approach for now: reduce safe speed on downhill grades.
        grade_factor = 1.0
        if grade_at_max_curvature < 0: # Downhill
            # Reduce safe speed for downhill curves. For example, a 5% grade (approx 0.05 rad)
            # might reduce the effective lateral acceleration by a small percentage.
            # This is a placeholder and needs empirical tuning.
            grade_factor = max(0.8, 1.0 + grade_at_max_curvature * 5.0) # Reduce by up to 20% for steep downhill
        elif grade_at_max_curvature > 0: # Uphill
            # For uphill, we might slightly increase the safe speed, or keep it neutral for conservatism.
            # For now, let's keep it neutral to prioritize safety.
            grade_factor = 1.0

        effective_max_lat_accel = self.max_lat_accel * grade_factor
        safe_speed = (effective_max_lat_accel / max_curvature_ahead) ** 0.5 if max_curvature_ahead > 0.0001 else car_state_msg.vEgo

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

  def calculate_overall_safety_score(self, car_state_msg, driver_monitoring_state_msg) -> float:
    """Calculate overall safety score based on all inputs"""
    # Weighted combination of all confidence measures
    # Adjust weights based on environmental conditions
    # These weights are chosen based on the assumed reliability and criticality of each sensor
    # in an autonomous driving context. While initial values are set, these require
    # extensive empirical testing and real-world validation to ensure optimal balance
    # and performance across diverse driving scenarios.
    # - Model (0.4): High weight due to its comprehensive understanding of the driving scene.
    #   Assumes the model's outputs are generally robust and accurate.
    # - Camera (0.3): Significant weight for lane detection, object classification, and visual cues.
    #   Reliability can be affected by lighting and weather conditions.
    # - Radar (0.2): Moderate weight for precise distance and velocity measurements, especially for lead vehicles.
    #   More robust in adverse weather than camera, but has a narrower field of view.
    # - IMU (0.1): Lower weight for general vehicle dynamics and environmental condition estimation.
    #   Provides foundational vehicle state but is a proxy for complex environmental factors.
    base_weights = {
      'model': 0.4,
      'camera': 0.3,
      'radar': 0.2,
      'imu': 0.1
    }

    # Apply sensor health status to confidence values
    model_conf = self.model_confidence if self.model_healthy else 0.1
    radar_conf = self.radar_confidence if self.radar_healthy else 0.1
    camera_conf = self.camera_confidence if self.camera_healthy else 0.1
    imu_conf = self.imu_confidence if self.imu_healthy else 0.1 # Currently IMU confidence is always 1.0 but this prepares for future degradation

    # Adjust weights based on environmental conditions
    if self.lighting_condition in ["night", "dawn_dusk"]: # Removed 'tunnel' as detection is not yet reliable.
      base_weights['model'] *= 0.8  # Reduce camera-based confidence in poor lighting
      base_weights['radar'] *= 1.1  # Increase radar confidence

    if self.weather_condition in ["rain", "snow", "fog", "unknown"]: # Apply conservative weights if weather is unknown
      base_weights['model'] *= 0.7
      base_weights['camera'] *= 0.6
      base_weights['radar'] *= 1.2  # Radar more reliable in poor weather

    # Calculate base safety score
    safety_score = (
      model_conf * base_weights['model'] +
      camera_conf * base_weights['camera'] +
      radar_conf * base_weights['radar'] +
      imu_conf * base_weights['imu']
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

    # Apply driver monitoring penalty
    if not self.driver_monitoring_healthy:
      logging.warning("Driver monitoring system unhealthy. Applying significant safety score penalty.")
      safety_score *= 0.5 # Significant penalty if system is not working
    elif driver_monitoring_state_msg is not None and hasattr(driver_monitoring_state_msg, 'awarenessStatus') and driver_monitoring_state_msg.awarenessStatus is not None:
      # Assuming awarenessStatus is 0-1, where low is bad
      if driver_monitoring_state_msg.awarenessStatus < 0.3: # Critically low awareness
        logging.warning(f"Critically low driver awareness ({driver_monitoring_state_msg.awarenessStatus:.2f}). Applying significant safety score penalty.")
        safety_score *= 0.6
      elif driver_monitoring_state_msg.awarenessStatus < 0.5: # Moderate low awareness
        logging.warning(f"Low driver awareness ({driver_monitoring_state_msg.awarenessStatus:.2f}). Applying moderate safety score penalty.")
        safety_score *= 0.8
    
    return max(0.0, min(1.0, safety_score))

  def evaluate_safety_intervention_needed(self, safety_score: float, car_state_msg, driver_monitoring_state_msg) -> bool:
    """Determine if safety intervention is required"""
    # Check multiple conditions for intervention
    intervention_needed = False
    
    # Model confidence too low
    if self.model_confidence < self.model_confidence_threshold * self.model_confidence_threshold_multiplier:
      intervention_needed = True
    
    # If any critical sensor is unhealthy, intervention is more likely
    if not self.model_healthy or not self.radar_healthy or not self.camera_healthy:
        logging.warning("Critical sensor unhealthy, increasing likelihood of intervention.")
        intervention_needed = True # Even if other scores are good, sensor failure is critical
    
    # Driver monitoring: critical awareness or unhealthy system leads to intervention
    if not self.driver_monitoring_healthy:
      logging.warning("Driver monitoring system unhealthy. Intervention needed.")
      intervention_needed = True
    elif driver_monitoring_state_msg is not None and hasattr(driver_monitoring_state_msg, 'awarenessStatus') and driver_monitoring_state_msg.awarenessStatus is not None:
      if driver_monitoring_state_msg.awarenessStatus < 0.2: # Very low awareness, immediate intervention
        logging.warning(f"Very low driver awareness ({driver_monitoring_state_msg.awarenessStatus:.2f}). Intervention needed.")
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

  def update(self, model_v2_msg, model_v2_mono_time, radar_state_msg, radar_state_mono_time, car_state_msg, car_state_mono_time, car_control_msg, live_pose_msg=None, live_pose_mono_time=None, driver_monitoring_state_msg=None, driver_monitoring_state_mono_time=None) -> Tuple[float, bool, Dict]:
    """Main update function - processes all inputs and returns safety assessment"""
    current_time = time.monotonic() * 1e9 # Get current time once in nanoseconds. time.monotonic returns seconds.
    start_time_update = time.monotonic() # Start timing for the entire update method

    # --- Initialize healthy flags to True at the start of each update cycle ---
    # They will be set to False if data is stale or an exception occurs during processing
    self.model_healthy = True
    self.radar_healthy = True
    self.camera_healthy = True
    self.imu_healthy = True

    # --- Staleness Checks ---
    # Convert mono_time to seconds for comparison
    # Model Staleness
    if (current_time * 1e-9 - model_v2_mono_time * 1e-9) > self.STALENESS_THRESHOLD_SECONDS:
      self.model_healthy = False
      logging.warning(f"Model data is stale. Last update: {(current_time - model_v2_mono_time) * 1e-9:.2f}s ago. Confidence reduced.")
    
    # Radar Staleness
    if (current_time * 1e-9 - radar_state_mono_time * 1e-9) > self.STALENESS_THRESHOLD_SECONDS:
      self.radar_healthy = False
      logging.warning(f"Radar data is stale. Last update: {(current_time - radar_state_mono_time) * 1e-9:.2f}s ago. Confidence reduced.")

    # Camera (carState is used as a proxy for freshness of car data influencing camera confidence)
    if (current_time * 1e-9 - car_state_mono_time * 1e-9) > self.STALENESS_THRESHOLD_SECONDS:
      self.camera_healthy = False
      logging.warning(f"CarState data (influencing camera confidence) is stale. Last update: {(current_time - car_state_mono_time) * 1e-9:.2f}s ago. Confidence reduced.")

    # IMU Staleness (livePose)
    if live_pose_mono_time is not None and (current_time * 1e-9 - live_pose_mono_time * 1e-9) > self.STALENESS_THRESHOLD_SECONDS:
      self.imu_healthy = False
      logging.warning(f"IMU data (livePose) is stale. Last update: {(current_time - live_pose_mono_time) * 1e-9:.2f}s ago. Confidence reduced.")
    elif live_pose_mono_time is None: # If live_pose_msg is None, then IMU data is not available
      self.imu_healthy = False
      logging.warning("LivePose message is not available. IMU data confidence reduced.")

        

    # Driver Monitoring Staleness
    if driver_monitoring_state_mono_time is not None and (current_time * 1e-9 - driver_monitoring_state_mono_time * 1e-9) > self.STALENESS_THRESHOLD_SECONDS:
      self.driver_monitoring_healthy = False
      logging.warning(f"Driver Monitoring data is stale. Last update: {(current_time - driver_monitoring_state_mono_time) * 1e-9:.2f}s ago. Confidence reduced.")
    elif driver_monitoring_state_mono_time is None:
      self.driver_monitoring_healthy = False
      logging.warning("Driver Monitoring message is not available. Driver monitoring confidence reduced.")

    

        # --- Update Confidence Measures with Error Handling ---
    try:
      # If already marked unhealthy due to staleness, no need to update confidence values explicitly,
      # as `calculate_overall_safety_score` will handle it.
      # Only update if the sensor is still considered healthy at this point (not stale).
      if self.model_healthy:
        self.update_model_confidence(model_v2_msg)
      else:
        self.model_confidence = 0.1 # Set low confidence if stale
    except Exception as e:
      logging.error(f"Error in update_model_confidence: {e}")
      self.model_confidence = 0.5  # Default to moderate confidence on error
      self.model_healthy = False
    
    try:
      if self.radar_healthy:
        self.update_radar_confidence(radar_state_msg, car_state_msg)
      else:
        self.radar_confidence = 0.1 # Set low confidence if stale
    except Exception as e:
      logging.error(f"Error in update_radar_confidence: {e}")
      self.radar_confidence = 0.5
      self.radar_healthy = False

    try:
      if self.camera_healthy:
        self.update_camera_confidence(model_v2_msg, car_state_msg)
      else:
        self.camera_confidence = 0.1 # Set low confidence if stale
    except Exception as e:
      logging.error(f"Error in update_camera_confidence: {e}")
      self.camera_confidence = 0.5
      self.camera_healthy = False

    try:
      if self.imu_healthy: # Only update environmental conditions if IMU is healthy
        self.detect_environmental_conditions(model_v2_msg, car_state_msg, car_control_msg, live_pose_msg)
      else: # If IMU is unhealthy, set environmental conditions to unknown and confidence low
        self.lighting_condition = "unknown"
        self.weather_condition = "unknown"
        self.road_condition = "unknown"
        self.imu_confidence = 0.1 # Already set if stale, but ensure if an error occurs here
    except Exception as e:
      logging.error(f"Error in detect_environmental_conditions: {e}")
      self.imu_healthy = False
      self.imu_confidence = 0.1 # Set IMU confidence to low on processing error
      # Also set environmental conditions to unknown on error
      self.lighting_condition = "unknown"
      self.weather_condition = "unknown"
      self.road_condition = "unknown"


      # Calculate overall safety with error handling
      try:
        self.overall_safety_score = self.calculate_overall_safety_score(car_state_msg, driver_monitoring_state_msg)
      except Exception as e:
        import logging
        logging.error(f"Error in calculate_overall_safety_score: {e}")
        self.overall_safety_score = 0.5

      try:
        self.requires_intervention = self.evaluate_safety_intervention_needed(
          self.overall_safety_score, car_state_msg, driver_monitoring_state_msg
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
        'model_healthy': self.model_healthy,
        'radar_healthy': self.radar_healthy,
        'camera_healthy': self.camera_healthy,
        'imu_healthy': self.imu_healthy,
        'driver_monitoring_healthy': self.driver_monitoring_healthy,
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