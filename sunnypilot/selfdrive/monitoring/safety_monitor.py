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
from openpilot.common.params import Params


class SafetyMonitor:
  """
  Enhanced Safety Monitoring System for Autonomous Driving
  Implements multi-sensor fusion validation, confidence thresholds, and environmental adaptation
  """
  
  def __init__(self):
    # Initialize filters for smooth monitoring
    self.model_confidence_filter = FirstOrderFilter(1.0, 0.5, DT_MDL)
    self.radar_confidence_filter = FirstOrderFilter(1.0, 0.5, DT_MDL)
    self.lane_deviation_filter = FirstOrderFilter(0.0, 0.5, DT_MDL)
    
    # Confidence thresholds for safety decisions
    self.model_confidence_threshold = 0.7
    self.radar_confidence_threshold = 0.6
    self.lane_deviation_threshold = 0.8  # meters from center
    
    # Environmental condition detection
    self.lighting_condition = "normal"  # "night", "dawn_dusk", "normal", "tunnel"
    self.weather_condition = "clear"    # "clear", "rain", "snow", "fog"
    self.road_condition = "dry"         # "dry", "wet", "icy", "snow"
    
    # Sensor fusion confidence scores
    self.camera_confidence = 1.0
    self.radar_confidence = 1.0
    self.imu_confidence = 1.0
    
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
    else:
      self.model_confidence = 0.5  # Fallback to moderate confidence

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

  def detect_environmental_conditions(self, model_v2_msg, car_state_msg, car_control_msg) -> None:
    """Detect environmental conditions and adjust safety accordingly"""
    # Lighting condition detection based on camera input (simplified)
    if hasattr(model_v2_msg, 'frameId') and model_v2_msg.frameId > 0:
      # In real implementation, this would use actual camera data analysis
      # For simulation purposes, we'll use some model outputs
      if hasattr(model_v2_msg, 'meta') and hasattr(model_v2_msg.meta, 'lighting'):
        lighting_metric = model_v2_msg.meta.lighting if model_v2_msg.meta.lighting else 0.5
        if lighting_metric < 0.2:
          self.lighting_condition = "night"
        elif lighting_metric < 0.4:
          self.lighting_condition = "dawn_dusk"
        elif lighting_metric > 0.9:
          self.lighting_condition = "tunnel"
        else:
          self.lighting_condition = "normal"

    # Weather condition detection (simplified)
    if hasattr(car_control_msg, 'orientationNED') and len(car_control_msg.orientationNED) >= 3:
      # Use IMU data to detect potential weather conditions (e.g., excessive vibrations)
      pitch = car_control_msg.orientationNED[1]
      roll = car_control_msg.orientationNED[0]
      # In real implementation, this would use more sophisticated weather detection
      if abs(pitch) > 0.2 or abs(roll) > 0.15:  # Unusual road conditions
        self.road_condition = "wet"  # Could indicate wet/icy road
        self.weather_condition = "rain"
      else:
        self.road_condition = "dry"
        self.weather_condition = "clear"

  def detect_curve_anticipation(self, model_v2_msg, car_state_msg) -> None:
    """Enhanced curve anticipation with improved safety margins"""
    if hasattr(model_v2_msg, 'path') and len(model_v2_msg.path.x) > 10:
      # Analyze path curvature ahead
      max_curvature_ahead = 0.0
      curve_points_ahead = min(25, len(model_v2_msg.path.y))
      
      for i in range(5, curve_points_ahead):  # Look ahead starting from 0.5s
        if i < len(model_v2_msg.path.y):
          curvature = abs(model_v2_msg.path.y[i])
          max_curvature_ahead = max(max_curvature_ahead, curvature)
      
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
    if self.model_confidence < self.model_confidence_threshold * 0.8:
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
    if car_state_msg.vEgo < 5.0 and safety_score < 0.4:
      intervention_needed = True
    
    return intervention_needed

  def update(self, model_v2_msg, radar_state_msg, car_state_msg, car_control_msg) -> Tuple[float, bool, Dict]:
    """Main update function - processes all inputs and returns safety assessment"""
    # Update all confidence measures
    self.update_model_confidence(model_v2_msg)
    self.update_radar_confidence(radar_state_msg)
    self.update_camera_confidence(model_v2_msg, car_state_msg)
    self.detect_environmental_conditions(model_v2_msg, car_state_msg, car_control_msg)
    self.detect_curve_anticipation(model_v2_msg, car_state_msg)
    
    # Calculate overall safety
    self.overall_safety_score = self.calculate_overall_safety_score(car_state_msg)
    self.requires_intervention = self.evaluate_safety_intervention_needed(
      self.overall_safety_score, car_state_msg
    )
    
    # Determine if confidence is degraded (requires more conservative driving)
    self.confidence_degraded = self.overall_safety_score < 0.6
    
    # Prepare safety report
    safety_report = {
      'model_confidence': self.model_confidence,
      'radar_confidence': self.radar_confidence,
      'camera_confidence': self.camera_confidence,
      'imu_confidence': self.imu_confidence,
      'lighting_condition': self.lighting_condition,
      'weather_condition': self.weather_condition,
      'road_condition': self.road_condition,
      'curve_anticipation_active': self.curve_anticipation_active,
      'curve_anticipation_score': self.curve_anticipation_score,
      'lane_deviation': self.lane_deviation_filter.x,
      'overall_safety_score': self.overall_safety_score,
      'confidence_degraded': self.confidence_degraded
    }
    
    self.monitoring_cycles += 1
    
    return self.overall_safety_score, self.requires_intervention, safety_report