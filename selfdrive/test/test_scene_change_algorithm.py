#!/usr/bin/env python3
"""
Unit tests for the scene change detection algorithm from modeld.py
Testing the core algorithm in isolation to validate safety and performance.
"""

import numpy as np
import pytest


def scene_change_algorithm(current_frame, prev_frame, frame_skip_counter=0):
  """
  Isolated implementation of the scene change detection algorithm from modeld.py
  Returns: (should_run_model: bool, new_skip_counter: int, scene_changed: bool)
  """
  # Run model periodically to ensure we don't miss important scene changes
  # Max skip of 3 frames (at 20Hz this means minimum 5Hz inference)
  if frame_skip_counter >= 3:
    return True, 0, False  # Run model and reset counter

  # Check if frames have same shape
  if current_frame.shape != prev_frame.shape:
    return True, 0, True  # Different shapes - run model and reset

  # Use a subsampled version for performance - check every 8th pixel
  h_step = max(1, current_frame.shape[0] // 16)
  w_step = max(1, current_frame.shape[1] // 16)

  current_sample = current_frame[::h_step, ::w_step]
  prev_sample = prev_frame[::h_step, ::w_step]

  # Calculate mean absolute difference between frames
  diff = np.mean(np.abs(current_sample.astype(np.int16) - prev_sample.astype(np.int16)))

  # Threshold for scene change (adjustable based on testing)
  scene_change_threshold = 3.0  # Lower values = more aggressive skipping

  scene_changed = diff > scene_change_threshold

  if scene_changed:
    # Scene changed significantly, run model and reset counter
    return True, 0, True
  else:
    # Scene unchanged, increment skip counter and skip this frame
    return False, frame_skip_counter + 1, False


class TestSceneChangeAlgorithm:
  """Test the isolated scene change detection algorithm"""

  def test_identical_frames_skip(self):
    """Test that identical frames are skipped"""
    frame1 = np.full((480, 640, 3), 100, dtype=np.uint8)
    frame2 = np.full((480, 640, 3), 100, dtype=np.uint8)

    should_run, new_counter, scene_changed = scene_change_algorithm(
      frame2, frame1, frame_skip_counter=0)

    assert not should_run, "Identical frames should be skipped"
    assert not scene_changed, "Identical frames should not be marked as changed"
    assert new_counter == 1, "Skip counter should increment"

  def test_different_frames_run(self):
    """Test that significantly different frames trigger model run"""
    frame1 = np.full((480, 640, 3), 100, dtype=np.uint8)
    frame2 = np.full((480, 640, 3), 150, dtype=np.uint8)  # Different enough to exceed threshold

    should_run, new_counter, scene_changed = scene_change_algorithm(
      frame2, frame1, frame_skip_counter=0)

    assert should_run, "Different frames should trigger model run"
    assert scene_changed, "Different frames should be marked as changed"
    assert new_counter == 0, "Counter should reset when scene changes"

  def test_frame_skip_limit(self):
    """Test that max skip limit forces model run"""
    frame1 = np.full((480, 640, 3), 100, dtype=np.uint8)
    frame2 = np.full((480, 640, 3), 100, dtype=np.uint8)  # Same frame

    # Should run when counter reaches 3
    should_run, new_counter, scene_changed = scene_change_algorithm(
      frame2, frame1, frame_skip_counter=3)

    assert should_run, "Should run model when max skip reached"
    assert new_counter == 0, "Counter should reset after max skip"

  def test_moderately_different_frames(self):
    """Test frames with moderate differences around threshold"""
    frame1 = np.full((480, 640, 3), 100, dtype=np.uint8)
    # Add some noise to make it moderately different
    frame2 = np.full((480, 640, 3), 102, dtype=np.uint8)

    should_run, new_counter, scene_changed = scene_change_algorithm(
      frame2, frame1, frame_skip_counter=0)

    # With threshold of 3.0, 2 unit difference should NOT trigger scene change
    assert not should_run, "Moderately different frames should be skipped"
    assert not scene_changed, "Moderately different frames should not be marked as changed"

  def test_different_shape_frames(self):
    """Test that frames with different shapes trigger model run"""
    frame1 = np.full((480, 640, 3), 100, dtype=np.uint8)
    frame2 = np.full((240, 320, 3), 100, dtype=np.uint8)  # Different shape

    should_run, new_counter, scene_changed = scene_change_algorithm(
      frame2, frame1, frame_skip_counter=2)

    assert should_run, "Different shaped frames should trigger model run"
    assert scene_changed, "Different shaped frames should be marked as changed"
    assert new_counter == 0, "Counter should reset for different shapes"

  def test_algorithm_sensitivity(self):
    """Test to understand the algorithm's sensitivity"""
    # Create two frames with known small difference
    frame1 = np.random.randint(100, 101, (480, 640, 3), dtype=np.uint8)
    frame2 = np.random.randint(102, 103, (480, 640, 3), dtype=np.uint8)

    # Manually calculate what we expect
    h_step = max(1, 480 // 16)  # = 30
    w_step = max(1, 640 // 16)  # = 40

    current_sample = frame2[::h_step, ::w_step]  # Subsample
    prev_sample = frame1[::h_step, ::w_step]    # Subsample

    expected_diff = np.mean(np.abs(current_sample.astype(np.int16) - prev_sample.astype(np.int16)))

    should_run, new_counter, scene_changed = scene_change_algorithm(
      frame2, frame1, frame_skip_counter=0)

    # The algorithm should handle this appropriately based on the threshold
    if expected_diff > 3.0:
      assert should_run, f"Expected run due to diff {expected_diff} > 3.0"
    else:
      assert not should_run, f"Expected skip due to diff {expected_diff} <= 3.0"


class TestPerformanceMetrics:
  """Test performance aspects of the scene change algorithm"""

  def test_algorithm_efficiency(self):
    """Test that the algorithm runs efficiently"""
    import time

    # Create test frames
    frame1 = np.full((480, 640, 3), 100, dtype=np.uint8)
    frame2 = np.full((480, 640, 3), 105, dtype=np.uint8)

    # Time multiple runs
    start_time = time.monotonic()
    for _ in range(100):
      scene_change_algorithm(frame2, frame1, frame_skip_counter=0)
    end_time = time.monotonic()

    avg_time = (end_time - start_time) / 100
    # The algorithm should be fast (much less than frame time)
    assert avg_time < 0.01, f"Algorithm should be fast, took {avg_time}s per call"


def validate_edge_cases():
  """Validate the algorithm handles edge cases safely"""
  # Very small frames
  small_frame1 = np.full((16, 16, 3), 100, dtype=np.uint8)
  small_frame2 = np.full((16, 16, 3), 105, dtype=np.uint8)

  should_run, new_counter, scene_changed = scene_change_algorithm(
    small_frame2, small_frame1, frame_skip_counter=0)

  # Should handle small frames without errors
  assert isinstance(should_run, bool)
  assert isinstance(new_counter, int)
  assert isinstance(scene_changed, bool)


if __name__ == "__main__":
  raise RuntimeError("pytest.main is banned, run with `pytest {__file__}` instead")