"""
Trigger-Based Learning Module for sunnypilot Firehose Mode
===========================================================

This module implements intelligent data tagging and prioritization for the
Firehose data upload system. Instead of passively uploading all data, it
automatically identifies and prioritizes "Failure Case" segments for training.

Key Features:
- Auto-detection of user disengagements during E2E maneuvers
- Failure case tagging with context (why did user disengage?)
- Priority scoring for training data selection
- Self-healing feedback loop: failures → training → improvement

Improvements for "Perfect Grade" E2E:
- Active Data Firehose: Trigger-based learning on disengagement
- Auto-tags segments as "Failure Cases" for prioritized training
- Creates self-healing loop: system learns from its mistakes
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Any
from collections import deque
from enum import IntEnum
import time
import json


class DisengagementReason(IntEnum):
    """Categorizes why a disengagement occurred"""
    UNKNOWN = 0
    USER_OVERRIDE_STEER = 1      # User grabbed steering wheel
    USER_OVERRIDE_BRAKE = 2      # User hit brake
    USER_OVERRIDE_GAS = 3        # User pressed gas
    SYSTEM_ALERT = 4             # System requested disengagement
    LANE_DEVIATION = 5           # Vehicle drifted from lane
    EXCESSIVE_ACCEL = 6          # Jerky acceleration/braking
    UNCERTAINTY_HIGH = 7         # Model uncertainty too high
    COLLISION_RISK = 8           # Potential collision detected
    TRAFFIC_SIGNAL = 9           # Stop sign/red light handling
    CURVE_SPEED = 10             # Entering curve too fast
    CUT_IN = 11                  # Lead vehicle cut in
    MERGE_CONFUSION = 12         # Lane change/merge confusion


@dataclass
class DisengagementEvent:
    """Records a single disengagement event for training"""
    timestamp: float
    reason: DisengagementReason
    severity: float  # 0.0-1.0, how abrupt was the disengagement?

    # Context at time of disengagement
    v_ego: float
    steering_angle: float
    torque_command: float
    model_confidence: float
    model_uncertainty: float

    # E2E specific
    e2e_active: bool
    e2e_mode: str
    e2e_torque_command: float
    e2e_confidence: float

    # Environment
    has_lead: bool
    lead_distance: float
    lane_curvature: float
    road_type: str  # highway, urban, residential

    # Tags for training
    tags: list[str] = field(default_factory=list)

    # Upload priority (higher = more important for training)
    upload_priority: float = 0.5

    # Segment metadata
    segment_id: str = ""
    start_frame: int = 0
    end_frame: int = 0

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for JSON serialization"""
        return {
            'timestamp': self.timestamp,
            'reason': self.reason.name,
            'severity': self.severity,
            'v_ego': self.v_ego,
            'steering_angle': self.steering_angle,
            'torque_command': self.torque_command,
            'model_confidence': self.model_confidence,
            'model_uncertainty': self.model_uncertainty,
            'e2e_active': self.e2e_active,
            'e2e_mode': self.e2e_mode,
            'e2e_torque_command': self.e2e_torque_command,
            'e2e_confidence': self.e2e_confidence,
            'has_lead': self.has_lead,
            'lead_distance': self.lead_distance,
            'lane_curvature': self.lane_curvature,
            'road_type': self.road_type,
            'tags': self.tags,
            'upload_priority': self.upload_priority,
            'segment_id': self.segment_id,
            'start_frame': self.start_frame,
            'end_frame': self.end_frame
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'DisengagementEvent':
        """Create from dictionary"""
        return cls(
            timestamp=data['timestamp'],
            reason=DisengagementReason[data['reason']],
            severity=data['severity'],
            v_ego=data['v_ego'],
            steering_angle=data['steering_angle'],
            torque_command=data['torque_command'],
            model_confidence=data['model_confidence'],
            model_uncertainty=data['model_uncertainty'],
            e2e_active=data['e2e_active'],
            e2e_mode=data['e2e_mode'],
            e2e_torque_command=data['e2e_torque_command'],
            e2e_confidence=data['e2e_confidence'],
            has_lead=data['has_lead'],
            lead_distance=data['lead_distance'],
            lane_curvature=data['lane_curvature'],
            road_type=data['road_type'],
            tags=data['tags'],
            upload_priority=data['upload_priority'],
            segment_id=data['segment_id'],
            start_frame=data['start_frame'],
            end_frame=data['end_frame']
        )


class TriggerBasedFirehose:
    """
    A+ Enhancement: Trigger-Based Learning for Firehose Mode
    
    This system automatically detects and tags disengagement events
    during E2E maneuvers, prioritizing them for training data uploads.
    
    The goal is to create a self-healing loop:
    1. User disengages during E2E maneuver
    2. System auto-tags segment as "Failure Case"
    3. Segment gets uploaded with high priority
    4. Model is retrained on failure cases
    5. System improves and avoids similar failures
    """

    # Time window to capture before/after disengagement (seconds)
    PRE_DISENGAGEMENT_WINDOW = 5.0
    POST_DISENGAGEMENT_WINDOW = 3.0

    # Thresholds for disengagement detection
    STEER_OVERRIDE_THRESHOLD = 5.0  # Nm
    BRAKE_OVERRIDE_THRESHOLD = 0.3  # pedal position
    GAS_OVERRIDE_THRESHOLD = 0.3    # pedal position

    # Priority multipliers
    PRIORITY_E2E_FAILURE = 2.0      # E2E-specific failures get 2x priority
    PRIORITY_HIGH_SEVERITY = 1.5    # Severe disengagements get 1.5x
    PRIORITY_RARE_SCENARIO = 1.3    # Rare scenarios get 1.3x

    def __init__(self,
                 enable_auto_tagging: bool = True,
                 enable_priority_scoring: bool = True,
                 max_events_buffer: int = 1000):
        self.enable_auto_tagging = enable_auto_tagging
        self.enable_priority_scoring = enable_priority_scoring

        # Event buffer for recent disengagements
        self.events_buffer: deque = deque(maxlen=max_events_buffer)

        # Context history for capturing pre-disengagement data
        self.context_history: deque = deque(maxlen=200)  # 10 seconds at 20Hz

        # Statistics
        self.total_disengagements = 0
        self.e2e_disengagements = 0
        self.uploaded_events = 0

        # Scenario frequency tracking (for rare scenario detection)
        self.scenario_counts: dict[str, int] = {}

        # Current segment tracking
        self.current_segment_id = ""
        self.current_frame = 0
        self.last_disengagement_time = 0.0

    def update_context(self,
                      timestamp: float,
                      v_ego: float,
                      steering_angle: float,
                      torque_command: float,
                      model_confidence: float,
                      model_uncertainty: float,
                      e2e_active: bool,
                      e2e_mode: str,
                      e2e_torque_command: float,
                      e2e_confidence: float,
                      has_lead: bool,
                      lead_distance: float,
                      lane_curvature: float,
                      road_type: str):
        """
        Record current context for potential disengagement tagging
        
        This should be called every frame to maintain context history.
        """
        context = {
            'timestamp': timestamp,
            'v_ego': v_ego,
            'steering_angle': steering_angle,
            'torque_command': torque_command,
            'model_confidence': model_confidence,
            'model_uncertainty': model_uncertainty,
            'e2e_active': e2e_active,
            'e2e_mode': e2e_mode,
            'e2e_torque_command': e2e_torque_command,
            'e2e_confidence': e2e_confidence,
            'has_lead': has_lead,
            'lead_distance': lead_distance,
            'lane_curvature': lane_curvature,
            'road_type': road_type,
            'frame': self.current_frame
        }

        self.context_history.append(context)
        self.current_frame += 1

    def detect_disengagement(self,
                            timestamp: float,
                            user_steer_override: bool,
                            user_brake_override: bool,
                            user_gas_override: bool,
                            system_request_disengage: bool,
                            lane_deviation: float,
                            excessive_accel: bool,
                            collision_risk: float) -> Optional[DisengagementReason]:
        """
        Detect disengagement and determine the reason
        
        Args:
            timestamp: Current timestamp
            user_steer_override: True if user applied steering torque
            user_brake_override: True if user pressed brake
            user_gas_override: True if user pressed gas
            system_request_disengage: True if system requested disengage
            lane_deviation: Lateral deviation from lane (meters)
            excessive_accel: True if acceleration was excessive
            collision_risk: Collision risk probability
            
        Returns:
            DisengagementReason if disengagement detected, None otherwise
        """
        if user_steer_override:
            return DisengagementReason.USER_OVERRIDE_STEER
        elif user_brake_override:
            return DisengagementReason.USER_OVERRIDE_BRAKE
        elif user_gas_override:
            return DisengagementReason.USER_OVERRIDE_GAS
        elif system_request_disengage:
            return DisengagementReason.SYSTEM_ALERT
        elif abs(lane_deviation) > 0.5:
            return DisengagementReason.LANE_DEVIATION
        elif excessive_accel:
            return DisengagementReason.EXCESSIVE_ACCEL
        elif collision_risk > 0.7:
            return DisengagementReason.COLLISION_RISK

        return None

    def create_disengagement_event(self,
                                   reason: DisengagementReason,
                                   timestamp: float) -> DisengagementEvent:
        """
        Create a disengagement event with full context
        
        Args:
            reason: Reason for disengagement
            timestamp: Time of disengagement
            
        Returns:
            DisengagementEvent with captured context
        """
        # Find context at disengagement time
        disengage_context = self._find_context_at_time(timestamp)

        if disengage_context is None:
            # Use most recent context if exact match not found
            disengage_context = self.context_history[-1] if self.context_history else {}

        # Calculate severity based on vehicle state
        severity = self._calculate_severity(
            disengage_context,
            reason
        )

        # Generate tags
        tags = self._generate_tags(reason, disengage_context)

        # Determine road type
        road_type = disengage_context.get('road_type', 'unknown')

        # Create event
        event = DisengagementEvent(
            timestamp=timestamp,
            reason=reason,
            severity=severity,
            v_ego=disengage_context.get('v_ego', 0.0),
            steering_angle=disengage_context.get('steering_angle', 0.0),
            torque_command=disengage_context.get('torque_command', 0.0),
            model_confidence=disengage_context.get('model_confidence', 0.0),
            model_uncertainty=disengage_context.get('model_uncertainty', 0.0),
            e2e_active=disengage_context.get('e2e_active', False),
            e2e_mode=disengage_context.get('e2e_mode', 'unknown'),
            e2e_torque_command=disengage_context.get('e2e_torque_command', 0.0),
            e2e_confidence=disengage_context.get('e2e_confidence', 0.0),
            has_lead=disengage_context.get('has_lead', False),
            lead_distance=disengage_context.get('lead_distance', 0.0),
            lane_curvature=disengage_context.get('lane_curvature', 0.0),
            road_type=road_type,
            tags=tags,
            segment_id=self.current_segment_id,
            start_frame=max(0, disengage_context.get('frame', 0) - int(self.PRE_DISENGAGEMENT_WINDOW * 20)),
            end_frame=disengage_context.get('frame', 0) + int(self.POST_DISENGAGEMENT_WINDOW * 20)
        )

        # Calculate upload priority
        event.upload_priority = self._calculate_upload_priority(event)

        # Update statistics
        self.total_disengagements += 1
        if event.e2e_active:
            self.e2e_disengagements += 1

        # Track scenario frequency
        scenario_key = f"{road_type}_{reason.name}"
        self.scenario_counts[scenario_key] = self.scenario_counts.get(scenario_key, 0) + 1

        # Store event
        self.events_buffer.append(event)
        self.last_disengagement_time = timestamp

        return event

    def _find_context_at_time(self, timestamp: float) -> Optional[dict[str, Any]]:
        """Find context closest to given timestamp"""
        if not self.context_history:
            return None

        # Binary search could be used for large buffers, but linear is fine for 200 items
        closest = min(self.context_history,
                     key=lambda c: abs(c.get('timestamp', timestamp) - timestamp))
        return closest

    def _calculate_severity(self, context: dict[str, Any], reason: DisengagementReason) -> float:
        """
        Calculate severity of disengagement (0.0-1.0)
        
        Higher severity for:
        - High speed
        - Abrupt maneuvers
        - High uncertainty
        - Dangerous scenarios
        """
        severity = 0.3  # Base severity

        # Speed factor
        v_ego = context.get('v_ego', 0.0)
        if v_ego > 20:  # > 72 km/h
            severity += 0.2
        elif v_ego > 10:  # > 36 km/h
            severity += 0.1

        # Uncertainty factor
        uncertainty = context.get('model_uncertainty', 0.0)
        if uncertainty > 0.8:
            severity += 0.2
        elif uncertainty > 0.5:
            severity += 0.1

        # Reason-specific severity
        if reason in (DisengagementReason.COLLISION_RISK, DisengagementReason.LANE_DEVIATION):
            severity += 0.3
        elif reason in (DisengagementReason.EXCESSIVE_ACCEL, DisengagementReason.USER_OVERRIDE_STEER):
            severity += 0.2

        return min(severity, 1.0)

    def _generate_tags(self, reason: DisengagementReason, context: dict[str, Any]) -> list[str]:
        """Generate descriptive tags for the disengagement"""
        tags = []

        # E2E-specific tags
        if context.get('e2e_active', False):
            tags.append('e2e_active')
            tags.append(f"e2e_mode_{context.get('e2e_mode', 'unknown')}")

        # Road type tags
        road_type = context.get('road_type', 'unknown')
        tags.append(road_type)

        # Speed tags
        v_ego = context.get('v_ego', 0.0)
        if v_ego > 25:
            tags.append('high_speed')
        elif v_ego < 5:
            tags.append('low_speed')

        # Reason tags
        tags.append(reason.name.lower())

        # Traffic tags
        if context.get('has_lead', False):
            tags.append('has_lead')
            lead_dist = context.get('lead_distance', 0.0)
            if lead_dist < 10:
                tags.append('close_lead')

        # Curvature tags
        curvature = context.get('lane_curvature', 0.0)
        if abs(curvature) > 0.01:
            tags.append('curve')

        return tags

    def _calculate_upload_priority(self, event: DisengagementEvent) -> float:
        """
        Calculate upload priority (0.0-1.0)
        
        Higher priority for:
        - E2E failures
        - High severity
        - Rare scenarios
        - High uncertainty
        """
        if not self.enable_priority_scoring:
            return 0.5

        priority = 0.3  # Base priority

        # E2E failure bonus
        if event.e2e_active:
            priority *= self.PRIORITY_E2E_FAILURE

        # Severity bonus
        if event.severity > 0.7:
            priority *= self.PRIORITY_HIGH_SEVERITY

        # Rare scenario bonus
        scenario_key = f"{event.road_type}_{event.reason.name}"
        scenario_count = self.scenario_counts.get(scenario_key, 0)
        if scenario_count <= 2:
            priority *= self.PRIORITY_RARE_SCENARIO

        # Uncertainty bonus
        if event.model_uncertainty > 0.7:
            priority *= 1.2

        # Confidence mismatch bonus (high confidence but disengaged)
        if event.e2e_confidence > 0.8:
            priority *= 1.3

        return min(priority, 1.0)

    def get_events_for_upload(self,
                             max_events: int = 10,
                             min_priority: float = 0.5) -> list[DisengagementEvent]:
        """
        Get events prioritized for upload
        
        Args:
            max_events: Maximum number of events to return
            min_priority: Minimum priority threshold
            
        Returns:
            List of events sorted by priority
        """
        # Filter by priority
        eligible_events = [
            e for e in self.events_buffer
            if e.upload_priority >= min_priority
        ]

        # Sort by priority (descending)
        sorted_events = sorted(eligible_events,
                              key=lambda e: e.upload_priority,
                              reverse=True)

        return sorted_events[:max_events]

    def mark_event_uploaded(self, event: DisengagementEvent):
        """Mark an event as successfully uploaded"""
        self.uploaded_events += 1
        # Could add 'uploaded' tag to event for tracking

    def get_statistics(self) -> dict[str, Any]:
        """Get statistics about disengagements"""
        e2e_rate = 0.0
        if self.total_disengagements > 0:
            e2e_rate = self.e2e_disengagements / self.total_disengagements

        # Get most common scenarios
        sorted_scenarios = sorted(
            self.scenario_counts.items(),
            key=lambda x: x[1],
            reverse=True
        )[:5]

        return {
            'total_disengagements': self.total_disengagements,
            'e2e_disengagements': self.e2e_disengagements,
            'e2e_disengagement_rate': e2e_rate,
            'uploaded_events': self.uploaded_events,
            'events_in_buffer': len(self.events_buffer),
            'most_common_scenarios': sorted_scenarios,
            'avg_priority': float(np.mean([e.upload_priority for e in self.events_buffer])) if self.events_buffer else 0.0
        }

    def export_events_to_json(self, filepath: str, min_priority: float = 0.5):
        """Export events to JSON file for upload"""
        events = self.get_events_for_upload(min_priority=min_priority)

        data = {
            'version': 1,
            'export_timestamp': time.time(),
            'statistics': self.get_statistics(),
            'events': [e.to_dict() for e in events]
        }

        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

        return len(events)

    def set_current_segment(self, segment_id: str):
        """Set current segment ID for tagging"""
        self.current_segment_id = segment_id
        self.current_frame = 0


# Convenience functions for integration
_default_firehose: Optional[TriggerBasedFirehose] = None


def get_trigger_firehose() -> TriggerBasedFirehose:
    """Get or create default TriggerBasedFirehose instance"""
    global _default_firehose
    if _default_firehose is None:
        _default_firehose = TriggerBasedFirehose()
    return _default_firehose


def record_disengagement(reason: DisengagementReason, timestamp: float) -> Optional[DisengagementEvent]:
    """
    Convenience function to record a disengagement
    
    Args:
        reason: Reason for disengagement
        timestamp: Time of disengagement
        
    Returns:
        Created DisengagementEvent
    """
    firehose = get_trigger_firehose()
    return firehose.create_disengagement_event(reason, timestamp)


def update_context(**kwargs):
    """Convenience function to update context"""
    firehose = get_trigger_firehose()
    firehose.update_context(
        timestamp=kwargs.get('timestamp', time.time()),
        v_ego=kwargs.get('v_ego', 0.0),
        steering_angle=kwargs.get('steering_angle', 0.0),
        torque_command=kwargs.get('torque_command', 0.0),
        model_confidence=kwargs.get('model_confidence', 0.0),
        model_uncertainty=kwargs.get('model_uncertainty', 0.0),
        e2e_active=kwargs.get('e2e_active', False),
        e2e_mode=kwargs.get('e2e_mode', 'unknown'),
        e2e_torque_command=kwargs.get('e2e_torque_command', 0.0),
        e2e_confidence=kwargs.get('e2e_confidence', 0.0),
        has_lead=kwargs.get('has_lead', False),
        lead_distance=kwargs.get('lead_distance', 0.0),
        lane_curvature=kwargs.get('lane_curvature', 0.0),
        road_type=kwargs.get('road_type', 'unknown')
    )
