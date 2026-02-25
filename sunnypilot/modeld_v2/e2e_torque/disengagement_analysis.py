"""
Disengagement Analysis and Telemetry Tool
==========================================

This module implements automated disengagement analysis for sunnypilot.

Key Features:
- Automatic detection of disengagement events
- Saves last 30 seconds of World Model imagined paths
- Compares model's planned trajectory vs human actual trajectory
- Auto-tunes blended_weight based on disengagement patterns
- Telemetry recording for offline analysis
- Root cause classification (planning, perception, control, comfort)

This achieves Recommendation #5: "Edge-Case Telemetry"
by providing automated tools to learn from disengagements.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List, Tuple
from collections import deque
from enum import Enum
import json
import time
import os


class DisengagementReason(Enum):
    """Classification of disengagement reasons"""
    PLANNING = "planning"  # Model planned unsafe/undesirable path
    PERCEPTION = "perception"  # Model missed obstacle/hazard
    CONTROL = "control"  # Model control was jerky/uncomfortable
    COMFORT = "comfort"  # Model behavior was uncomfortable but safe
    USER_REQUEST = "user_request"  # User manually disengaged
    SYSTEM_FAULT = "system_fault"  # System error or fault
    UNKNOWN = "unknown"


@dataclass
class TrajectorySnapshot:
    """Snapshot of a trajectory at a point in time"""
    timestamp: float
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    is_collision: bool
    uncertainty: float
    cost: float


@dataclass
class WorldModelSnapshot:
    """Snapshot of World Model imagination"""
    timestamp: float
    imagined_trajectories: List[TrajectorySnapshot]
    chosen_trajectory_idx: int
    mppi_weights: Optional[np.ndarray]
    expected_cost: float
    entropy: float


@dataclass
class DisengagementEvent:
    """
    Complete record of a disengagement event
    
    Contains 30 seconds of before/after data for analysis
    """
    # Event metadata
    event_id: str
    timestamp: float
    duration: float
    reason: DisengagementReason
    severity: float  # 0-1, how severe was the disengagement
    
    # Vehicle state at disengagement
    vehicle_speed: float
    steering_angle: float
    lateral_accel: float
    longitude: float
    latitude: float
    road_type: str
    
    # World Model data
    world_model_history: List[WorldModelSnapshot] = field(default_factory=list)
    
    # Human trajectory (what the human actually did)
    human_trajectory: Optional[TrajectorySnapshot] = None
    
    # Model trajectory (what the model planned)
    model_trajectory: Optional[TrajectorySnapshot] = None
    
    # Analysis results
    trajectory_divergence: float = 0.0  # How different was human vs model
    root_cause: Optional[str] = None
    suggested_fix: Optional[str] = None
    
    # Telemetry
    telemetry_data: Dict[str, Any] = field(default_factory=dict)
    
    # Learning
    blended_weight_adjustment: float = 0.0  # Suggested adjustment to E2E weight


class TelemetryRecorder:
    """
    Continuous telemetry recorder for disengagement analysis
    
    Maintains rolling buffer of:
    - World Model snapshots (imagined trajectories)
    - Vehicle state
    - Control commands
    - Sensor data
    
    When disengagement occurs, saves last 30 seconds for analysis.
    """
    
    def __init__(self,
                 history_seconds: float = 30.0,
                 sample_rate_hz: float = 20.0,
                 save_dir: str = "/tmp/sunnypilot/disengagements"):
        """
        Initialize telemetry recorder
        
        Args:
            history_seconds: How much history to save on disengagement
            sample_rate_hz: Recording sample rate
            save_dir: Directory to save disengagement data
        """
        self.history_seconds = history_seconds
        self.sample_rate_hz = sample_rate_hz
        self.save_dir = save_dir
        
        # Compute buffer sizes
        self.buffer_size = int(history_seconds * sample_rate_hz)
        
        # Rolling buffers
        self.world_model_buffer: deque = deque(maxlen=self.buffer_size)
        self.vehicle_state_buffer: deque = deque(maxlen=self.buffer_size)
        self.control_buffer: deque = deque(maxlen=self.buffer_size)
        self.timestamps: deque = deque(maxlen=self.buffer_size)
        
        # Event counter
        self.event_count = 0
        
        # Ensure save directory exists
        os.makedirs(save_dir, exist_ok=True)
    
    def record_world_model(self,
                          world_model_snapshot: WorldModelSnapshot):
        """
        Record World Model snapshot
        
        Args:
            world_model_snapshot: Current World Model state
        """
        self.world_model_buffer.append(world_model_snapshot)
        self.timestamps.append(world_model_snapshot.timestamp)
    
    def record_vehicle_state(self,
                            speed: float,
                            steering_angle: float,
                            lateral_accel: float,
                            yaw_rate: float,
                            longitude: float,
                            latitude: float,
                            road_type: str = "unknown"):
        """
        Record vehicle state
        
        Args:
            speed: Vehicle speed (m/s)
            steering_angle: Steering angle (rad)
            lateral_accel: Lateral acceleration (m/s^2)
            yaw_rate: Yaw rate (rad/s)
            longitude: GPS longitude
            latitude: GPS latitude
            road_type: Type of road
        """
        self.vehicle_state_buffer.append({
            'speed': speed,
            'steering_angle': steering_angle,
            'lateral_accel': lateral_accel,
            'yaw_rate': yaw_rate,
            'longitude': longitude,
            'latitude': latitude,
            'road_type': road_type,
            'timestamp': self.timestamps[-1] if self.timestamps else time.time()
        })
    
    def record_control(self,
                      torque_command: float,
                      torque_actual: float,
                      curvature_command: float,
                      curvature_actual: float,
                      blended_weight: float):
        """
        Record control commands and actuals
        
        Args:
            torque_command: Commanded torque
            torque_actual: Actual torque (from sensors)
            curvature_command: Commanded curvature
            curvature_actual: Actual curvature
            blended_weight: E2E vs fallback blend weight
        """
        self.control_buffer.append({
            'torque_command': torque_command,
            'torque_actual': torque_actual,
            'curvature_command': curvature_command,
            'curvature_actual': curvature_actual,
            'blended_weight': blended_weight,
            'timestamp': self.timestamps[-1] if self.timestamps else time.time()
        })
    
    def capture_disengagement(self,
                             reason: DisengagementReason,
                             severity: float = 0.5) -> DisengagementEvent:
        """
        Capture disengagement event
        
        Saves all buffered data and creates DisengagementEvent.
        
        Args:
            reason: Reason for disengagement
            severity: Severity of disengagement (0-1)
        
        Returns:
            DisengagementEvent with captured data
        """
        self.event_count += 1
        
        # Generate event ID
        event_id = f"disengage_{self.event_count}_{int(time.time())}"
        
        # Get current timestamp
        current_time = self.timestamps[-1] if self.timestamps else time.time()
        
        # Get vehicle state at disengagement
        vehicle_state = self.vehicle_state_buffer[-1] if self.vehicle_state_buffer else {}
        
        # Build World Model history
        world_model_history = list(self.world_model_buffer)
        
        # Compute trajectory divergence (model vs human)
        trajectory_divergence = self._compute_trajectory_divergence(
            world_model_history, vehicle_state
        )
        
        # Create event
        event = DisengagementEvent(
            event_id=event_id,
            timestamp=current_time,
            duration=self.history_seconds,
            reason=reason,
            severity=severity,
            vehicle_speed=vehicle_state.get('speed', 0.0),
            steering_angle=vehicle_state.get('steering_angle', 0.0),
            lateral_accel=vehicle_state.get('lateral_accel', 0.0),
            longitude=vehicle_state.get('longitude', 0.0),
            latitude=vehicle_state.get('latitude', 0.0),
            road_type=vehicle_state.get('road_type', 'unknown'),
            world_model_history=world_model_history,
            trajectory_divergence=trajectory_divergence
        )
        
        # Analyze event
        self._analyze_disengagement(event)
        
        # Save event to disk
        self._save_event(event)
        
        return event
    
    def _compute_trajectory_divergence(self,
                                       world_model_history: List[WorldModelSnapshot],
                                       vehicle_state: Dict[str, Any]) -> float:
        """
        Compute how much the human trajectory diverged from model trajectory
        
        Args:
            world_model_history: History of World Model snapshots
            vehicle_state: Current vehicle state
        
        Returns:
            Divergence metric (higher = more different)
        """
        if not world_model_history:
            return 0.0
        
        # Get model's chosen trajectory from most recent snapshot
        latest_snapshot = world_model_history[-1]
        if not latest_snapshot.imagined_trajectories:
            return 0.0
        
        chosen_traj = latest_snapshot.imagined_trajectories[
            latest_snapshot.chosen_trajectory_idx
        ]
        
        # Compare model's planned position to actual vehicle position
        model_pos = chosen_traj.positions[0]  # First point (immediate future)
        actual_pos = np.array([
            vehicle_state.get('longitude', 0.0),
            vehicle_state.get('latitude', 0.0),
            0.0
        ])
        
        # Compute divergence (simplified - would need proper coordinate transform)
        divergence = float(np.linalg.norm(model_pos[:2] - actual_pos[:2]))
        
        return divergence
    
    def _analyze_disengagement(self, event: DisengagementEvent):
        """
        Analyze disengagement to determine root cause and suggested fix
        
        Args:
            event: Disengagement event to analyze
        """
        # Analyze World Model history
        if event.world_model_history:
            latest = event.world_model_history[-1]
            
            # Check for high uncertainty
            if latest.entropy > 2.0:
                event.root_cause = "high_uncertainty"
                event.suggested_fix = "Increase planning horizon or reduce speed"
            
            # Check for high cost
            if latest.expected_cost > 50.0:
                event.root_cause = "high_cost_trajectories"
                event.suggested_fix = "Review cost function weights"
            
            # Check for collision in chosen trajectory
            if (latest.imagined_trajectories and
                latest.chosen_trajectory_idx < len(latest.imagined_trajectories)):
                chosen = latest.imagined_trajectories[latest.chosen_trajectory_idx]
                if chosen.is_collision:
                    event.root_cause = "collision_planned"
                    event.suggested_fix = "Improve collision detection or planning"
        
        # Analyze by reason
        if event.reason == DisengagementReason.PLANNING:
            event.blended_weight_adjustment = -0.1  # Reduce E2E weight
            event.suggested_fix = "Reduce E2E blend weight, increase fallback"
        
        elif event.reason == DisengagementReason.CONTROL:
            event.blended_weight_adjustment = -0.05
            event.suggested_fix = "Review torque smoothing and rate limiting"
        
        elif event.reason == DisengagementReason.COMFORT:
            event.blended_weight_adjustment = -0.02
            event.suggested_fix = "Adjust comfort weights in cost function"
        
        elif event.reason == DisengagementReason.PERCEPTION:
            event.root_cause = "perception_failure"
            event.suggested_fix = "Review radar/vision fusion for missed objects"
        
        # Default
        if event.root_cause is None:
            event.root_cause = "unknown"
            event.suggested_fix = "Manual review required"
    
    def _save_event(self, event: DisengagementEvent):
        """
        Save disengagement event to disk
        
        Args:
            event: Event to save
        """
        # Create save directory for this event
        event_dir = os.path.join(self.save_dir, event.event_id)
        os.makedirs(event_dir, exist_ok=True)
        
        # Save metadata as JSON
        metadata = {
            'event_id': event.event_id,
            'timestamp': event.timestamp,
            'duration': event.duration,
            'reason': event.reason.value,
            'severity': event.severity,
            'vehicle_speed': event.vehicle_speed,
            'steering_angle': event.steering_angle,
            'lateral_accel': event.lateral_accel,
            'longitude': event.longitude,
            'latitude': event.latitude,
            'road_type': event.road_type,
            'trajectory_divergence': event.trajectory_divergence,
            'root_cause': event.root_cause,
            'suggested_fix': event.suggested_fix,
            'blended_weight_adjustment': event.blended_weight_adjustment
        }
        
        metadata_path = os.path.join(event_dir, 'metadata.json')
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        # Save World Model trajectories
        trajectories_data = []
        for snapshot in event.world_model_history:
            snapshot_data = {
                'timestamp': snapshot.timestamp,
                'chosen_idx': snapshot.chosen_trajectory_idx,
                'expected_cost': snapshot.expected_cost,
                'entropy': snapshot.entropy,
                'trajectories': []
            }
            
            for traj in snapshot.imagined_trajectories:
                traj_data = {
                    'positions': traj.positions.tolist(),
                    'velocities': traj.velocities.tolist(),
                    'accelerations': traj.accelerations.tolist(),
                    'is_collision': traj.is_collision,
                    'uncertainty': traj.uncertainty,
                    'cost': traj.cost
                }
                snapshot_data['trajectories'].append(traj_data)
            
            trajectories_data.append(snapshot_data)
        
        trajectories_path = os.path.join(event_dir, 'trajectories.json')
        with open(trajectories_path, 'w') as f:
            json.dump(trajectories_data, f, indent=2)
        
        # Save telemetry
        telemetry = {
            'vehicle_states': list(self.vehicle_state_buffer),
            'controls': list(self.control_buffer)
        }
        
        telemetry_path = os.path.join(event_dir, 'telemetry.json')
        with open(telemetry_path, 'w') as f:
            json.dump(telemetry, f, indent=2)
    
    def get_recent_events(self, n: int = 10) -> List[Dict[str, Any]]:
        """
        Get summary of recent disengagement events
        
        Args:
            n: Number of events to return
        
        Returns:
            List of event summaries
        """
        # Scan save directory for events
        events = []
        
        if not os.path.exists(self.save_dir):
            return events
        
        for event_id in sorted(os.listdir(self.save_dir))[-n:]:
            event_dir = os.path.join(self.save_dir, event_id)
            metadata_path = os.path.join(event_dir, 'metadata.json')
            
            if os.path.exists(metadata_path):
                with open(metadata_path, 'r') as f:
                    events.append(json.load(f))
        
        return events
    
    def clear_buffer(self):
        """Clear all buffers"""
        self.world_model_buffer.clear()
        self.vehicle_state_buffer.clear()
        self.control_buffer.clear()
        self.timestamps.clear()


class DisengagementAnalyzer:
    """
    Analyzer for disengagement patterns and auto-tuning
    
    Analyzes historical disengagement data to:
    1. Identify common failure modes
    2. Auto-tune blended_weight for E2E vs fallback
    3. Generate reports for developers
    4. Suggest model improvements
    """
    
    def __init__(self,
                 telemetry_recorder: Optional[TelemetryRecorder] = None,
                 auto_tune_enabled: bool = True,
                 min_events_for_tuning: int = 10):
        """
        Initialize disengagement analyzer
        
        Args:
            telemetry_recorder: TelemetryRecorder instance
            auto_tune_enabled: Enable automatic tuning
            min_events_for_tuning: Minimum events before auto-tuning
        """
        self.telemetry_recorder = telemetry_recorder or TelemetryRecorder()
        self.auto_tune_enabled = auto_tune_enabled
        self.min_events_for_tuning = min_events_for_tuning
        
        # Historical events
        self.events: List[DisengagementEvent] = []
        
        # Current blended weight
        self.current_blended_weight = 0.5  # 50% E2E, 50% fallback
        
        # Tuning history
        self.tuning_history: List[Dict[str, Any]] = []
    
    def record_disengagement(self,
                            reason: DisengagementReason,
                            severity: float = 0.5) -> DisengagementEvent:
        """
        Record and analyze a disengagement event
        
        Args:
            reason: Reason for disengagement
            severity: Severity (0-1)
        
        Returns:
            DisengagementEvent
        """
        # Capture event from telemetry
        event = self.telemetry_recorder.capture_disengagement(reason, severity)
        
        # Store event
        self.events.append(event)
        
        # Auto-tune if enabled
        if self.auto_tune_enabled and len(self.events) >= self.min_events_for_tuning:
            self._auto_tune()
        
        return event
    
    def _auto_tune(self):
        """
        Auto-tune blended_weight based on disengagement patterns
        
        Analyzes recent events to determine optimal E2E vs fallback balance.
        """
        if not self.events:
            return
        
        # Analyze recent events
        recent_events = self.events[-self.min_events_for_tuning:]
        
        # Compute average adjustment
        adjustments = [e.blended_weight_adjustment for e in recent_events]
        avg_adjustment = np.mean(adjustments)
        
        # Apply adjustment
        old_weight = self.current_blended_weight
        self.current_blended_weight = np.clip(
            self.current_blended_weight + avg_adjustment,
            0.0,
            1.0
        )
        
        # Record tuning
        tuning_record = {
            'timestamp': time.time(),
            'old_weight': old_weight,
            'new_weight': self.current_blended_weight,
            'adjustment': avg_adjustment,
            'num_events': len(recent_events),
            'reason_counts': self._count_reasons(recent_events)
        }
        
        self.tuning_history.append(tuning_record)
        
        # Save tuning results
        self._save_tuning_history()
    
    def _count_reasons(self, events: List[DisengagementEvent]) -> Dict[str, int]:
        """Count disengagement reasons"""
        counts = {}
        for event in events:
            reason = event.reason.value
            counts[reason] = counts.get(reason, 0) + 1
        return counts
    
    def _save_tuning_history(self):
        """Save tuning history to disk"""
        save_path = os.path.join(
            self.telemetry_recorder.save_dir,
            'tuning_history.json'
        )
        
        with open(save_path, 'w') as f:
            json.dump(self.tuning_history, f, indent=2)
    
    def get_analysis_report(self) -> Dict[str, Any]:
        """
        Generate analysis report of all disengagements
        
        Returns:
            Analysis report dictionary
        """
        if not self.events:
            return {'error': 'No disengagement events recorded'}
        
        # Count reasons
        reason_counts = self._count_reasons(self.events)
        
        # Compute statistics
        severities = [e.severity for e in self.events]
        divergences = [e.trajectory_divergence for e in self.events]
        
        # Root cause distribution
        root_causes = {}
        for event in self.events:
            if event.root_cause:
                root_causes[event.root_cause] = root_causes.get(event.root_cause, 0) + 1
        
        # Common scenarios
        road_types = {}
        for event in self.events:
            road_types[event.road_type] = road_types.get(event.road_type, 0) + 1
        
        # Speed distribution
        speeds = [e.vehicle_speed for e in self.events]
        
        report = {
            'total_events': len(self.events),
            'reason_counts': reason_counts,
            'root_cause_distribution': root_causes,
            'road_type_distribution': road_types,
            'avg_severity': float(np.mean(severities)),
            'max_severity': float(np.max(severities)),
            'avg_trajectory_divergence': float(np.mean(divergences)),
            'avg_speed': float(np.mean(speeds)),
            'max_speed': float(np.max(speeds)),
            'current_blended_weight': self.current_blended_weight,
            'tuning_count': len(self.tuning_history),
            'recommendations': self._generate_recommendations()
        }
        
        return report
    
    def _generate_recommendations(self) -> List[str]:
        """Generate recommendations based on disengagement patterns"""
        recommendations = []
        
        if not self.events:
            return recommendations
        
        reason_counts = self._count_reasons(self.events)
        
        # Planning issues
        if reason_counts.get('planning', 0) > len(self.events) * 0.3:
            recommendations.append(
                "High rate of planning disengagements. Consider reducing E2E blend weight "
                "or improving cost function."
            )
        
        # Perception issues
        if reason_counts.get('perception', 0) > len(self.events) * 0.2:
            recommendations.append(
                "Perception failures detected. Review radar/vision fusion and "
                "occlusion handling in temporal memory."
            )
        
        # Control issues
        if reason_counts.get('control', 0) > len(self.events) * 0.2:
            recommendations.append(
                "Control issues detected. Consider tuning neural observer or "
                "adjusting torque rate limits."
            )
        
        # Comfort issues
        if reason_counts.get('comfort', 0) > len(self.events) * 0.15:
            recommendations.append(
                "Comfort complaints. Review cost function comfort weights and "
                "consider smoother trajectory generation."
            )
        
        # High speed disengagements
        high_speed_events = [e for e in self.events if e.vehicle_speed > 25.0]
        if len(high_speed_events) > len(self.events) * 0.3:
            recommendations.append(
                "Many disengagements at high speed. Consider more conservative "
                "planning at speeds > 25 m/s."
            )
        
        return recommendations
    
    def get_blended_weight(self) -> float:
        """Get current blended weight for E2E vs fallback"""
        return self.current_blended_weight
    
    def export_events(self, output_path: str):
        """
        Export all events to a file for offline analysis
        
        Args:
            output_path: Path to save exported data
        """
        export_data = {
            'events': [],
            'report': self.get_analysis_report(),
            'tuning_history': self.tuning_history
        }
        
        for event in self.events:
            event_data = {
                'event_id': event.event_id,
                'timestamp': event.timestamp,
                'reason': event.reason.value,
                'severity': event.severity,
                'vehicle_speed': event.vehicle_speed,
                'road_type': event.road_type,
                'trajectory_divergence': event.trajectory_divergence,
                'root_cause': event.root_cause,
                'suggested_fix': event.suggested_fix,
                'blended_weight_adjustment': event.blended_weight_adjustment
            }
            export_data['events'].append(event_data)
        
        with open(output_path, 'w') as f:
            json.dump(export_data, f, indent=2)


class DisengagementIntegrationHelper:
    """
    Helper class for integrating disengagement analysis with existing controls
    
    Provides hooks for:
    - Detecting disengagement triggers
    - Recording telemetry automatically
    - Querying blended weight from analyzer
    """
    
    def __init__(self,
                 enable_recording: bool = True,
                 enable_auto_tune: bool = True,
                 save_dir: str = "/tmp/sunnypilot/disengagements"):
        """
        Initialize disengagement integration
        
        Args:
            enable_recording: Enable telemetry recording
            enable_auto_tune: Enable automatic tuning
            save_dir: Directory to save disengagement data
        """
        self.enable_recording = enable_recording
        self.enable_auto_tune = enable_auto_tune
        
        # Initialize components
        self.telemetry_recorder = TelemetryRecorder(save_dir=save_dir)
        self.analyzer = DisengagementAnalyzer(
            telemetry_recorder=self.telemetry_recorder,
            auto_tune_enabled=enable_auto_tune
        )
        
        # Recording state
        self.is_recording = True
    
    def update(self,
              world_model_snapshot: Optional[WorldModelSnapshot],
              vehicle_state: Dict[str, Any],
              control_data: Dict[str, Any]):
        """
        Update telemetry with current data
        
        Call this at each control loop iteration.
        
        Args:
            world_model_snapshot: Current World Model state (optional)
            vehicle_state: Current vehicle state
            control_data: Current control commands
        """
        if not self.is_recording or not self.enable_recording:
            return
        
        # Record World Model
        if world_model_snapshot:
            self.telemetry_recorder.record_world_model(world_model_snapshot)
        
        # Record vehicle state
        self.telemetry_recorder.record_vehicle_state(
            speed=vehicle_state.get('speed', 0.0),
            steering_angle=vehicle_state.get('steering_angle', 0.0),
            lateral_accel=vehicle_state.get('lateral_accel', 0.0),
            yaw_rate=vehicle_state.get('yaw_rate', 0.0),
            longitude=vehicle_state.get('longitude', 0.0),
            latitude=vehicle_state.get('latitude', 0.0),
            road_type=vehicle_state.get('road_type', 'unknown')
        )
        
        # Record control
        self.telemetry_recorder.record_control(
            torque_command=control_data.get('torque_command', 0.0),
            torque_actual=control_data.get('torque_actual', 0.0),
            curvature_command=control_data.get('curvature_command', 0.0),
            curvature_actual=control_data.get('curvature_actual', 0.0),
            blended_weight=self.analyzer.get_blended_weight()
        )
    
    def on_disengagement(self,
                        reason: str,
                        severity: float = 0.5) -> DisengagementEvent:
        """
        Call when disengagement occurs
        
        Args:
            reason: Reason string (planning, perception, control, comfort, user_request)
            severity: Severity (0-1)
        
        Returns:
            DisengagementEvent
        """
        # Map reason string to enum
        reason_map = {
            'planning': DisengagementReason.PLANNING,
            'perception': DisengagementReason.PERCEPTION,
            'control': DisengagementReason.CONTROL,
            'comfort': DisengagementReason.COMFORT,
            'user_request': DisengagementReason.USER_REQUEST,
            'system_fault': DisengagementReason.SYSTEM_FAULT
        }
        
        reason_enum = reason_map.get(reason.lower(), DisengagementReason.UNKNOWN)
        
        # Record disengagement
        event = self.analyzer.record_disengagement(reason_enum, severity)
        
        return event
    
    def get_blended_weight(self) -> float:
        """Get current blended weight for E2E control"""
        return self.analyzer.get_blended_weight()
    
    def get_analysis_report(self) -> Dict[str, Any]:
        """Get disengagement analysis report"""
        return self.analyzer.get_analysis_report()
    
    def pause_recording(self):
        """Pause telemetry recording"""
        self.is_recording = False
    
    def resume_recording(self):
        """Resume telemetry recording"""
        self.is_recording = True
