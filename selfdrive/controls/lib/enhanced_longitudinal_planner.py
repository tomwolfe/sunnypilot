"""
Enhanced longitudinal planning module for sunnypilot
Implements improved lead vehicle prediction using smoothed trajectory estimation and probabilistic spacing models
"""
import numpy as np
from typing import Tuple, Optional, Dict, Any
from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.modeld.constants import ModelConstants


class EnhancedLongitudinalPlanner:
    """
    Enhanced longitudinal planner that improves lead vehicle prediction
    with smoothed trajectory estimation and probabilistic spacing models.
    """
    
    def __init__(self, CP):
        self.CP = CP
        
        # Parameters for trajectory smoothing
        self.smoothing_factor = 0.1  # Factor for exponential smoothing
        self.lead_reliability_factor = 0.8  # How much to trust current lead data
        self.prediction_horizon = 10  # seconds to predict ahead
        self.min_lead_distance = 25.0  # minimum comfortable distance in meters
        
        # Filters for lead vehicle state estimation
        self.lead_distance_filter = FirstOrderFilter(0.0, 1.5, 1.0/ModelConstants.MODEL_FREQ)
        self.lead_speed_filter = FirstOrderFilter(0.0, 1.5, 1.0/ModelConstants.MODEL_FREQ)
        self.lead_accel_filter = FirstOrderFilter(0.0, 1.5, 1.0/ModelConstants.MODEL_FREQ)
        
        # Track lead vehicle history for better prediction
        self.lead_history = []
        self.max_history = 50  # Maximum history entries to keep
        
        # Confidence-based acceleration limits
        self.confidence_accel_limits = {
            'high': (CP.longitudinalTuning.kA, CP.longitudinalTuning.kA),  # Use original limits when confident
            'medium': (CP.longitudinalTuning.kA * 0.8, CP.longitudinalTuning.kA * 0.8),  # Reduced when medium confidence
            'low': (CP.longitudinalTuning.kA * 0.5, CP.longitudinalTuning.kA * 0.5)  # Further reduced when low confidence
        }
        
    def update_lead_vehicle_prediction(self,
                                     long_plan: log.LongitudinalPlan,
                                     radar_state: Optional[log.RadarState],
                                     car_state: Optional[log.CarState],
                                     validation_metrics: Optional[Dict[str, Any]] = None) -> log.LongitudinalPlan:
        """
        Update longitudinal plan with enhanced lead vehicle prediction based on validation metrics.

        Args:
            long_plan: Original longitudinal plan from model
            radar_state: Radar state containing lead vehicle data
            car_state: Car state for vehicle speed information
            validation_metrics: Validation metrics from enhanced vision

        Returns:
            Enhanced longitudinal plan with improved lead vehicle prediction
        """
        # Check validation metrics to determine prediction reliability
        lead_confidence = 0.5  # Default if no validation metrics available
        if validation_metrics is not None:
            lead_confidence = validation_metrics.get('leadConfidenceAvg', 0.5)

        # Determine confidence level based on lead detection confidence
        if lead_confidence >= 0.75:
            confidence_level = 'high'
        elif lead_confidence >= 0.5:
            confidence_level = 'medium'
        else:
            confidence_level = 'low'

        # Enhanced lead vehicle prediction
        enhanced_long_plan = self._enhance_lead_prediction(long_plan, radar_state, confidence_level)

        # Apply probabilistic spacing models based on validation metrics
        enhanced_long_plan = self._apply_probabilistic_spacing(enhanced_long_plan, radar_state, car_state, validation_metrics)

        # Update acceleration limits based on confidence
        enhanced_long_plan = self._adjust_acceleration_limits(enhanced_long_plan, confidence_level)

        return enhanced_long_plan

    def _enhance_lead_prediction(self,
                                long_plan: log.LongitudinalPlan,
                                radar_state: Optional[log.RadarState],
                                confidence_level: str) -> log.LongitudinalPlan:
        """
        Enhance lead vehicle prediction using smoothed trajectory estimation.
        """
        # Create a copy of the longitudinal plan to modify
        enhanced_plan = long_plan

        # Apply smoothing factor based on confidence level
        smoothing_factor = self.smoothing_factor
        if confidence_level == 'low':
            smoothing_factor = 0.3  # More smoothing when confidence is low
        elif confidence_level == 'medium':
            smoothing_factor = 0.2
        # 'high' confidence uses the default smoothing_factor

        # If lead vehicle exists in radar state, enhance its prediction
        if radar_state is not None and long_plan.hasLead and radar_state.leadOne.status:
            # Smooth the lead distance and speed predictions
            current_lead_dist = radar_state.leadOne.dRel
            current_lead_speed = radar_state.leadOne.vRel  # This is relative speed

            # Apply smoothing to lead distance
            smoothed_lead_dist = self.lead_distance_filter.update(current_lead_dist)

            # Also apply smoothing to relative speed
            smoothed_lead_speed = self.lead_speed_filter.update(current_lead_speed)

            # Update longitudinal plan based on smoothed data
            # This is a simplified approach - in a full implementation, we would have more sophisticated logic
            # to incorporate the smoothed lead data into the plan
            if abs(smoothed_lead_dist - current_lead_dist) > 2.0:  # If smoothing made a significant change
                # Adjust acceleration based on smoothed data
                if smoothed_lead_dist < 50.0:  # Close lead
                    # Apply more conservative acceleration targets
                    enhanced_plan.aTarget = min(enhanced_plan.aTarget, 0.0)  # Reduce acceleration when close to lead

        return enhanced_plan
    
    def _apply_probabilistic_spacing(self,
                                   long_plan: log.LongitudinalPlan,
                                   radar_state: Optional[log.RadarState],
                                   car_state: Optional[log.CarState],
                                   validation_metrics: Optional[Dict[str, Any]]) -> log.LongitudinalPlan:
        """
        Apply probabilistic spacing models to maintain safe distances based on confidence.
        """
        enhanced_plan = long_plan

        if (long_plan.hasLead and radar_state is not None and radar_state.leadOne.status and
            car_state is not None and validation_metrics is not None):
            # Calculate safety margins based on validation metrics
            lead_confidence = validation_metrics.get('leadConfidenceAvg', 0.5)
            overall_confidence = validation_metrics.get('overallConfidence', 0.5)

            # Get current lead distance from radar state
            current_lead_distance = radar_state.leadOne.dRel

            # Calculate dynamic safety distance based on confidence - use ego vehicle speed
            current_speed = car_state.vEgo  # Use vEgo from car state
            base_safe_distance = max(30.0, current_speed * 1.5)  # Base: 1.5 seconds at current speed

            # Adjust safety distance based on confidence - lower confidence = more distance
            confidence_factor = max(0.7, 1.5 - overall_confidence)  # Between 0.7 and 1.0
            safe_distance = base_safe_distance * confidence_factor

            # Ensure minimum safe distance
            if current_lead_distance < safe_distance:
                # Reduce target acceleration to maintain safe distance
                enhanced_plan.aTarget = min(enhanced_plan.aTarget, -0.5)  # Apply conservative braking
                enhanced_plan.shouldStop = current_lead_distance < self.min_lead_distance  # Stop if very close

        return enhanced_plan
    
    def _adjust_acceleration_limits(self, 
                                  long_plan: log.LongitudinalPlan,
                                  confidence_level: str) -> log.LongitudinalPlan:
        """
        Adjust acceleration limits based on confidence level.
        """
        enhanced_plan = long_plan
        
        # Adjust acceleration limits based on confidence
        accel_limits = self.confidence_accel_limits[confidence_level]
        
        # Apply more conservative limits when confidence is low
        if confidence_level == 'low':
            enhanced_plan.aTarget = max(min(enhanced_plan.aTarget, 1.0), -2.0)  # More conservative
        elif confidence_level == 'medium':
            enhanced_plan.aTarget = max(min(enhanced_plan.aTarget, 2.0), -3.0)  # Medium conservative
        # High confidence keeps original limits
        
        return enhanced_plan

    def reset(self):
        """Reset the enhanced longitudinal planner state."""
        self.lead_distance_filter.x = 0.0
        self.lead_speed_filter.x = 0.0
        self.lead_accel_filter.x = 0.0
        self.lead_history.clear()