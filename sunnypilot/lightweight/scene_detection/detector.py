"""
Lightweight Scene Change Detection for Sunnypilot2
Based on the enhanced vision model concepts but optimized for hardware constraints.

This module implements efficient scene change detection to reduce computational load
by processing fewer frames when the scene is static.
"""

import numpy as np
from typing import Tuple, Optional
import cv2
from dataclasses import dataclass


@dataclass
class SceneState:
    """Simple scene state for change detection."""
    frame_id: int
    timestamp: float
    motion_level: float
    static_scene: bool


class LightweightSceneChangeDetector:
    """
    Lightweight scene change detection optimized for Snapdragon 845.

    Uses simple frame differencing to detect scene changes and reduce
    computational load when the scene is static.
    """

    def __init__(self,
                 motion_threshold: float = 0.05,
                 min_static_frames: int = 3,
                 max_dynamic_frames: int = 1):
        """
        Initialize the scene change detector.

        Args:
            motion_threshold: Threshold for motion detection (0.0-1.0)
            min_static_frames: Min frames to confirm static scene
            max_dynamic_frames: Max frames to process when static
        """
        self.motion_threshold = motion_threshold
        self.min_static_frames = min_static_frames
        self.max_dynamic_frames = max_dynamic_frames

        self.prev_frame_gray = None
        self.static_frame_count = 0
        self.dynamic_frame_count = 0
        self.current_state = False  # True if scene is static

    def detect_change(self, current_frame: np.ndarray) -> Tuple[bool, float]:
        """
        Detect if scene has changed compared to previous frame.

        Args:
            current_frame: Current frame as numpy array

        Returns:
            Tuple of (should_skip_processing, motion_level)
        """
        if self.prev_frame_gray is None:
            # First frame, set as reference and process
            if len(current_frame.shape) == 3:
                self.prev_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_RGB2GRAY)
            else:
                self.prev_frame_gray = current_frame.copy()
            return False, 0.0  # Process first frame

        # Convert current frame to grayscale if needed
        if len(current_frame.shape) == 3:
            current_gray = cv2.cvtColor(current_frame, cv2.COLOR_RGB2GRAY)
        else:
            current_gray = current_frame

        # Compute simple frame difference
        frame_diff = cv2.absdiff(self.prev_frame_gray, current_gray)
        motion_level = float(np.mean(frame_diff)) / 255.0  # Normalize to [0, 1]

        # Update state based on motion threshold
        is_moving = motion_level > self.motion_threshold

        if is_moving:
            # Scene is dynamic, reset static counter
            self.static_frame_count = 0
            self.current_state = False
            self.dynamic_frame_count = self.max_dynamic_frames  # Reset dynamic frame allowance
            should_process = True
        else:
            # Scene appears static
            self.static_frame_count += 1
            self.current_state = self.static_frame_count >= self.min_static_frames

            if self.current_state:
                # In static state, skip processing based on max_dynamic_frames
                if self.dynamic_frame_count > 0:
                    self.dynamic_frame_count -= 1
                    # Process this frame to maintain some updates
                    should_process = True
                else:
                    should_process = False
            else:
                should_process = True  # Still building up to static state, so process

        # Update reference frame
        self.prev_frame_gray = current_gray.copy()

        return not should_process, motion_level  # Return (skip, motion_level)

    def reset(self):
        """Reset the detector state."""
        self.prev_frame_gray = None
        self.static_frame_count = 0
        self.dynamic_frame_count = 0
        self.current_state = False


def create_scene_change_detector() -> LightweightSceneChangeDetector:
    """
    Factory function to create a scene change detector with optimized parameters
    for Snapdragon 845 hardware constraints.
    """
    return LightweightSceneChangeDetector(
        motion_threshold=0.05,      # Adjusted for hardware constraints
        min_static_frames=3,       # Fewer frames for quicker detection
        max_dynamic_frames=1       # Process some frames even in static scenes
    )