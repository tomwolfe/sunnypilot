"""
Shadow Mode Auto-Labeling for E2E Training
==========================================

This module implements on-device "Shadow Mode" evaluation for E2E training.

When the E2E model's predicted torque differs significantly from the 
human driver's torque, the system automatically flags those segments 
for priority upload/training - enabling a closed-loop training pipeline.

Key Features:
- Real-time comparison of E2E prediction vs human driver
- Automatic flagging of high-divergence segments
- Priority upload queue management
- On-device labeling with confidence scores
"""

import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Any
from enum import IntEnum


class DivergenceType(IntEnum):
    """Types of divergence between E2E and human driver."""
    NONE = 0
    MINOR = 1
    MODERATE = 2
    MAJOR = 3
    CRITICAL = 4


@dataclass
class LabeledSegment:
    """A segment labeled for training."""
    start_time: float
    end_time: float
    duration_ms: float
    divergence_type: DivergenceType
    divergence_score: float
    e2e_torque: float
    human_torque: float
    context: dict[str, Any]
    priority: float
    uploaded: bool = False


@dataclass
class ShadowModeSnapshot:
    """Single snapshot for shadow mode comparison."""
    timestamp: float
    e2e_torque: float
    e2e_uncertainty: float
    human_torque: float
    speed: float
    curvature: float
    road_type: int
    weather: int
    visibility: float


class TorqueDivergenceDetector:
    """
    Detects when E2E model predictions diverge from human driver.
    
    Uses multiple signals to determine if the divergence is meaningful:
    - Raw torque difference
    - Directional disagreement  
    - Temporal persistence
    - Context (curvature, speed, road type)
    """

    TORQUE_DIFF_THRESHOLDS = {
        DivergenceType.MINOR: 0.3,
        DivergenceType.MODERATE: 0.6,
        DivergenceType.MAJOR: 1.0,
        DivergenceType.CRITICAL: 1.5,
    }

    DIRECTION_WEIGHT = 0.3
    PERSISTENCE_WEIGHT = 0.4
    CONTEXT_WEIGHT = 0.3

    def __init__(self):
        self._history = deque(maxlen=100)
        self._divergence_history = deque(maxlen=50)
        self._last_divergence_time = 0

    def compute_divergence(self,
                          e2e_torque: float,
                          human_torque: float,
                          e2e_uncertainty: float = 0.0,
                          speed: float = 0.0,
                          curvature: float = 0.0) -> tuple[DivergenceType, float]:
        """
        Compute divergence between E2E and human driver.
        
        Returns:
            (divergence_type, divergence_score)
        """
        torque_diff = abs(e2e_torque - human_torque)

        direction_agree = (e2e_torque * human_torque) >= 0
        direction_penalty = 0.0 if direction_agree else self.DIRECTION_WEIGHT

        recent_divergences = list(self._divergence_history)[-10:]
        persistence = sum(recent_divergences) / max(len(recent_divergences), 1)

        context_factor = 1.0
        if speed > 20.0:
            context_factor *= 1.2
        if abs(curvature) > 0.01:
            context_factor *= 1.3

        raw_score = torque_diff * (1.0 + direction_penalty)
        persistence_score = persistence * self.PERSISTENCE_WEIGHT
        context_score = context_factor * self.CONTEXT_WEIGHT

        final_score = raw_score + persistence_score + context_score

        if e2e_uncertainty > 0.5:
            final_score *= (1.0 + e2e_uncertainty)

        divergence_type = DivergenceType.NONE
        for dtype, threshold in self.TORQUE_DIFF_THRESHOLDS.items():
            if final_score >= threshold:
                divergence_type = dtype

        self._divergence_history.append(final_score)

        if divergence_type != DivergenceType.NONE:
            self._last_divergence_time = time.time()

        return divergence_type, final_score

    def is_persistent_divergence(self, min_duration_ms: float = 500.0) -> bool:
        """Check if divergence has persisted for minimum duration."""
        if not self._divergence_history:
            return False

        recent = list(self._divergence_history)[-int(min_duration_ms / 100):]
        return len(recent) >= int(min_duration_ms / 100) and all(d > 0 for d in recent)


class ShadowModeEvaluator:
    """
    Shadow Mode Evaluator for E2E Auto-Labeling.
    
    Runs in parallel with the E2E controller, comparing model predictions
    against human driver input. Automatically flags segments for training.
    """

    MAX_SEGMENT_DURATION_MS = 10000
    MIN_SEGMENT_DURATION_MS = 200
    PRIORITY_THRESHOLD = 0.7
    UPLOAD_QUEUE_SIZE = 100

    def __init__(self, enabled: bool = True):
        self._enabled = enabled

        self._divergence_detector = TorqueDivergenceDetector()

        self._current_segment: Optional[LabeledSegment] = None
        self._segment_queue: deque = deque(maxlen=self.UPLOAD_QUEUE_SIZE)

        self._snapshot_buffer: deque = deque(maxlen=1000)

        self._total_segments_flagged = 0
        self._total_uploaded = 0

        self._stats = {
            'minor_count': 0,
            'moderate_count': 0,
            'major_count': 0,
            'critical_count': 0,
            'avg_divergence': 0.0,
        }

    def add_snapshot(self,
                    e2e_torque: float,
                    human_torque: float,
                    e2e_uncertainty: float = 0.0,
                    speed: float = 0.0,
                    curvature: float = 0.0,
                    road_type: int = 0,
                    weather: int = 0,
                    visibility: float = 1.0) -> None:
        """Add a snapshot for evaluation."""
        if not self._enabled:
            return

        snapshot = ShadowModeSnapshot(
            timestamp=time.time(),
            e2e_torque=e2e_torque,
            e2e_uncertainty=e2e_uncertainty,
            human_torque=human_torque,
            speed=speed,
            curvature=curvature,
            road_type=road_type,
            weather=weather,
            visibility=visibility
        )

        self._snapshot_buffer.append(snapshot)

        divergence_type, divergence_score = self._divergence_detector.compute_divergence(
            e2e_torque, human_torque, e2e_uncertainty, speed, curvature
        )

        self._update_stats(divergence_type, divergence_score)

        if divergence_type != DivergenceType.NONE:
            self._handle_divergence(divergence_type, divergence_score, snapshot)

    def _update_stats(self, divergence_type: DivergenceType, score: float) -> None:
        """Update running statistics."""
        if divergence_type == DivergenceType.MINOR:
            self._stats['minor_count'] += 1
        elif divergence_type == DivergenceType.MODERATE:
            self._stats['moderate_count'] += 1
        elif divergence_type == DivergenceType.MAJOR:
            self._stats['major_count'] += 1
        elif divergence_type == DivergenceType.CRITICAL:
            self._stats['critical_count'] += 1

        n = self._total_segments_flagged + 1
        self._stats['avg_divergence'] = (
            (self._stats['avg_divergence'] * (n - 1) + score) / n
        )

    def _handle_divergence(self,
                          divergence_type: DivergenceType,
                          score: float,
                          snapshot: ShadowModeSnapshot) -> None:
        """Handle detected divergence - start or extend segment."""
        current_time = time.time()

        if self._current_segment is None:
            self._current_segment = LabeledSegment(
                start_time=current_time,
                end_time=current_time,
                duration_ms=0,
                divergence_type=divergence_type,
                divergence_score=score,
                e2e_torque=snapshot.e2e_torque,
                human_torque=snapshot.human_torque,
                context={},
                priority=0.0
            )
        else:
            self._current_segment.end_time = current_time
            self._current_segment.duration_ms = (current_time - self._current_segment.start_time) * 1000

            if divergence_type > self._current_segment.divergence_type:
                self._current_segment.divergence_type = divergence_type

            self._current_segment.e2e_torque = (
                self._current_segment.e2e_torque * 0.7 + snapshot.e2e_torque * 0.3
            )
            self._current_segment.human_torque = (
                self._current_segment.human_torque * 0.7 + snapshot.human_torque * 0.3
            )

            self._current_segment.divergence_score = (
                self._current_segment.divergence_score * 0.8 + score * 0.2
            )

        if self._current_segment.duration_ms >= self.MIN_SEGMENT_DURATION_MS:
            self._finalize_segment()

    def _finalize_segment(self) -> None:
        """Finalize and queue current segment for upload."""
        if self._current_segment is None:
            return

        segment = self._current_segment

        if segment.duration_ms > self.MAX_SEGMENT_DURATION_MS:
            segment.duration_ms = self.MAX_SEGMENT_DURATION_MS

        priority = self._compute_priority(segment)
        segment.priority = priority

        if priority >= self.PRIORITY_THRESHOLD:
            self._segment_queue.append(segment)
            self._total_segments_flagged += 1

        self._current_segment = None

    def _compute_priority(self, segment: LabeledSegment) -> float:
        """Compute upload priority for segment."""
        type_weight = float(segment.divergence_type) / DivergenceType.CRITICAL

        duration_factor = min(segment.duration_ms / 2000.0, 1.0)

        score_factor = min(segment.divergence_score / 3.0, 1.0)

        priority = (type_weight * 0.4 + duration_factor * 0.3 + score_factor * 0.3)

        return min(priority, 1.0)

    def force_finalize(self) -> None:
        """Force finalize any open segment."""
        if self._current_segment is not None:
            if self._current_segment.duration_ms >= 100:
                self._finalize_segment()
            else:
                self._current_segment = None

    def get_pending_uploads(self, max_count: int = 10) -> list[LabeledSegment]:
        """Get pending segments for upload."""
        segments = sorted(
            list(self._segment_queue),
            key=lambda s: s.priority,
            reverse=True
        )
        return segments[:max_count]

    def mark_uploaded(self, segment: LabeledSegment) -> None:
        """Mark segment as uploaded."""
        for i, s in enumerate(self._segment_queue):
            if (s.start_time == segment.start_time and
                s.end_time == segment.end_time):
                self._segment_queue[i].uploaded = True
                self._total_uploaded += 1
                break

        self._segment_queue = deque(
            [s for s in self._segment_queue if not s.uploaded],
            maxlen=self.UPLOAD_QUEUE_SIZE
        )

    def get_stats(self) -> dict[str, Any]:
        """Get shadow mode statistics."""
        return {
            **self._stats,
            'total_flagged': self._total_segments_flagged,
            'total_uploaded': self._total_uploaded,
            'pending_uploads': len(self._segment_queue),
            'current_segment_open': self._current_segment is not None
        }

    def enable(self) -> None:
        """Enable shadow mode."""
        self._enabled = True

    def disable(self) -> None:
        """Disable shadow mode and finalize any open segments."""
        self.force_finalize()
        self._enabled = False

    @property
    def is_enabled(self) -> bool:
        return self._enabled


class FirehoseAutoLabeler:
    """
    High-level interface for Firehose Auto-Labeling.
    
    Integrates Shadow Mode with the Firehose upload system for
    closed-loop E2E training.
    """

    def __init__(self, params=None):
        self._params = params

        self._shadow_mode = ShadowModeEvaluator(enabled=True)

        self._auto_upload_enabled = True
        self._last_upload_time = 0
        self._upload_interval_sec = 60

    def update(self,
              e2e_torque: float,
              human_torque: float,
              e2e_uncertainty: float = 0.0,
              speed: float = 0.0,
              curvature: float = 0.0,
              road_type: int = 0,
              weather: int = 0,
              visibility: float = 1.0) -> None:
        """
        Update with new control data.
        
        Should be called every control cycle (10Hz recommended).
        """
        self._shadow_mode.add_snapshot(
            e2e_torque=e2e_torque,
            human_torque=human_torque,
            e2e_uncertainty=e2e_uncertainty,
            speed=speed,
            curvature=curvature,
            road_type=road_type,
            weather=weather,
            visibility=visibility
        )

        if self._auto_upload_enabled:
            self._check_auto_upload()

    def _check_auto_upload(self) -> None:
        """Check if automatic upload should be triggered."""
        current_time = time.time()

        if current_time - self._last_upload_time < self._upload_interval_sec:
            return

        pending = self._shadow_mode.get_pending_uploads(max_count=5)

        if pending:
            self._trigger_upload(pending)
            self._last_upload_time = current_time

    def _trigger_upload(self, segments: list[LabeledSegment]) -> None:
        """Trigger upload of segments to training pipeline."""
        if self._params:
            segment_data = {
                'segments': [
                    {
                        'start_time': s.start_time,
                        'end_time': s.end_time,
                        'divergence_type': int(s.divergence_type),
                        'divergence_score': s.divergence_score,
                        'priority': s.priority
                    }
                    for s in segments
                ]
            }

            import json
            self._params.put_nonblocking(
                "ShadowModePendingUploads",
                json.dumps(segment_data)
            )

    def get_shadow_mode_stats(self) -> dict[str, Any]:
        """Get shadow mode statistics."""
        return self._shadow_mode.get_stats()

    def enable(self) -> None:
        """Enable auto-labeling."""
        self._shadow_mode.enable()

    def disable(self) -> None:
        """Disable auto-labeling."""
        self._shadow_mode.disable()
