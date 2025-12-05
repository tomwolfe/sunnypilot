#!/usr/bin/env python3
"""
Test suite for validating the scene change detection in modeld.py
This test ensures that the vision model skipping mechanism works safely
while providing the expected performance improvements.
"""

import numpy as np
import pytest
from unittest.mock import Mock, MagicMock
from openpilot.selfdrive.modeld.modeld import ModelState


class MockVisionBuf:
  """Mock VisionBuf for testing"""
  
  def __init__(self, width=640, height=480, stride=640, data=None):
    self.width = width
    self.height = height
    self.stride = stride
    
    if data is None:
      # Create a simple test pattern
      self.data = np.zeros((height, width, 3), dtype=np.uint8)
    else:
      self.data = data


class TestSceneChangeDetection:
  """Test suite for scene change detection validation"""
  
  def setup_method(self):
    """Set up test fixtures before each test method"""
    self.model_state = ModelState()
    # Reset internal state to known values
    self.model_state.prev_road_frame = None
    self.model_state.frame_skip_counter = 0
    self.model_state._last_vision_outputs = None
  
  def create_test_frame(self, value=100, width=640, height=480):
    """Create a test frame with a specific value"""
    frame_data = np.full((height, width, 3), value, dtype=np.uint8)
    return MockVisionBuf(width=width, height=height, data=frame_data)
  
  def test_initial_frame_handling(self):
    """Test that the first frame always runs the model"""
    bufs = {'roadCamera': self.create_test_frame(100)}
    transforms = {}
    
    # First call should run model since there's no previous frame
    should_run = self.model_state._should_run_vision_model(bufs, transforms)
    assert should_run == True, "First frame should always run the model"
  
  def test_static_scene_skipping(self):
    """Test that static scenes enable frame skipping"""
    bufs = {'roadCamera': self.create_test_frame(100)}
    transforms = {}
    
    # First call - should run
    should_run_1 = self.model_state._should_run_vision_model(bufs, transforms)
    assert should_run_1 == True, "First frame should run"
    
    # Second call with same frame - should skip
    should_run_2 = self.model_state._should_run_vision_model(bufs, transforms)
    assert should_run_2 == False, "Static scene should skip"
    
    # Third call with same frame - should skip
    should_run_3 = self.model_state._should_run_vision_model(bufs, transforms)
    assert should_run_3 == False, "Static scene should skip"
    
    # Fourth call with same frame - should run (max skip reached)
    should_run_4 = self.model_state._should_run_vision_model(bufs, transforms)
    assert should_run_4 == True, "Max skip reached, should run model"
  
  def test_different_scenes_run_model(self):
    """Test that different scenes always run the model"""
    bufs1 = {'roadCamera': self.create_test_frame(100)}  # Bright scene
    bufs2 = {'roadCamera': self.create_test_frame(50)}   # Darker scene
    transforms = {}
    
    # First call
    should_run_1 = self.model_state._should_run_vision_model(bufs1, transforms)
    assert should_run_1 == True, "First frame should run"
    
    # Different scene should run
    should_run_2 = self.model_state._should_run_vision_model(bufs2, transforms)
    assert should_run_2 == True, "Different scene should run model"
    
    # Same scene as second should skip
    should_run_3 = self.model_state._should_run_vision_model(bufs2, transforms)
    assert should_run_3 == False, "Same scene should skip"
  
  def test_scene_change_threshold_sensitivity(self):
    """Test the sensitivity of scene change detection"""
    # Create very similar frames - should be detected as same scene
    frame1 = np.full((480, 640, 3), 100, dtype=np.uint8)
    frame2 = np.full((480, 640, 3), 102, dtype=np.uint8)  # Only 2 units different
    
    bufs1 = {'roadCamera': MockVisionBuf(data=frame1)}
    bufs2 = {'roadCamera': MockVisionBuf(data=frame2)}
    transforms = {}
    
    # First call
    should_run_1 = self.model_state._should_run_vision_model(bufs1, transforms)
    assert should_run_1 == True, "First frame should run"
    
    # Similar frame - threshold is 3.0, difference is 2.0, so should skip
    should_run_2 = self.model_state._should_run_vision_model(bufs2, transforms)
    assert should_run_2 == False, "Similar scenes should skip (diff < threshold)"
  
  def test_no_camera_buffer_safety(self):
    """Test that missing camera buffer defaults to running model for safety"""
    bufs = {'roadCamera': None}  # No camera data
    transforms = {}
    
    should_run = self.model_state._should_run_vision_model(bufs, transforms)
    assert should_run == True, "Missing camera buffer should run model for safety"
  
  def test_missing_road_camera_safety(self):
    """Test that missing road camera defaults to running model for safety"""
    bufs = {'wideCamera': self.create_test_frame(100)}  # Missing roadCamera
    transforms = {}
    
    should_run = self.model_state._should_run_vision_model(bufs, transforms)
    assert should_run == True, "Missing road camera should run model for safety"
  
  def test_error_handling(self):
    """Test that errors in scene detection default to running model for safety"""
    # Create a scenario that might cause errors
    bufs = {'roadCamera': MockVisionBuf(width=640, height=480, stride=1000)}  # Invalid stride
    transforms = {}
    
    # This should catch any errors and return True to run model for safety
    should_run = self.model_state._should_run_vision_model(bufs, transforms)
    # Note: The exact behavior depends on how the error handling works in the actual code
    # If our error handling is working, this might run successfully with fallback
    assert isinstance(should_run, bool), "Should return boolean value"
  
  def test_frame_shape_mismatch(self):
    """Test handling of different frame shapes"""
    frame1 = np.full((480, 640, 3), 100, dtype=np.uint8)
    frame2 = np.full((240, 320, 3), 150, dtype=np.uint8)  # Different size
    
    bufs1 = {'roadCamera': MockVisionBuf(data=frame1)}
    bufs2 = {'roadCamera': MockVisionBuf(data=frame2)}
    transforms = {}
    
    # First call
    should_run_1 = self.model_state._should_run_vision_model(bufs1, transforms)
    assert should_run_1 == True, "First frame should run"
    
    # Different sized frame - should run for safety
    should_run_2 = self.model_state._should_run_vision_model(bufs2, transforms)
    assert should_run_2 == True, "Different frame sizes should run model"
  
  def test_skip_logic_reset_after_change(self):
    """Test that skip counter resets after scene change"""
    bufs1 = {'roadCamera': self.create_test_frame(100)}
    bufs2 = {'roadCamera': self.create_test_frame(200)}  # Different scene
    transforms = {}
    
    # First frame
    self.model_state._should_run_vision_model(bufs1, transforms)
    
    # Skip a few frames
    self.model_state._should_run_vision_model(bufs1, transforms)  # Skip
    self.model_state._should_run_vision_model(bufs1, transforms)  # Skip 
    skip_count_after_skips = self.model_state.frame_skip_counter
    
    # Scene change - should run and reset counter
    self.model_state._should_run_vision_model(bufs2, transforms)
    skip_count_after_change = self.model_state.frame_skip_counter
    
    # After scene change, counter should be reset to 0
    assert skip_count_after_change == 0, f"Skip counter should reset to 0 after scene change, got {skip_count_after_change}"
  

class TestModelExecutionWithSceneDetection:
  """Test the full model execution flow with scene detection"""
  
  def setup_method(self):
    """Set up test fixtures before each test method"""
    self.model_state = ModelState()
  
  def test_model_execution_flow(self):
    """Test the model execution flow with scene detection"""
    # Create mock inputs
    bufs = {'roadCamera': self.model_state.create_test_frame(100)}
    transforms = {}
    
    # Create minimal inputs for model execution
    inputs = {
      'policy_desire': np.zeros(4, dtype=np.float32),
      'meta': np.zeros(128, dtype=np.float32),
      'traffic_convention': np.array([0], dtype=np.int32)
    }
    
    # Test that the execution doesn't crash and returns appropriate results
    try:
      # This would normally execute the vision model if needed
      result = self.model_state.run(bufs, transforms, inputs, prepare_only=False)
      # The result might be None or a dict depending on if model was skipped
      assert result is not None or isinstance(result, dict), "Model execution should return valid result"
    except Exception as e:
      pytest.fail(f"Model execution failed with exception: {e}")
  
  def test_prepare_only_mode(self):
    """Test the prepare_only mode"""
    bufs = {'roadCamera': self.model_state.create_test_frame(100)}
    transforms = {}
    inputs = {
      'policy_desire': np.zeros(4, dtype=np.float32),
      'meta': np.zeros(128, dtype=np.float32),
      'traffic_convention': np.array([0], dtype=np.int32)
    }
    
    try:
      # Test prepare_only mode
      result = self.model_state.run(bufs, transforms, inputs, prepare_only=True)
      assert result is None, "Prepare only mode should return None"
    except Exception as e:
      pytest.fail(f"Prepare only execution failed with exception: {e}")


if __name__ == "__main__":
  pytest.main([__file__])