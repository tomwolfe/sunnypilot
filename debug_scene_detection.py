"""
Debug/analysis of the scene change detection logic to understand the test results.
"""

import numpy as np
from sunnypilot.lightweight.scene_detection.detector import create_scene_change_detector

def debug_scene_detection():
    print("=== Debugging Scene Change Detection Logic ===")
    
    detector = create_scene_change_detector()
    
    # Test completely static scene - all frames identical
    print("\\nTesting static scene (all frames identical):")
    static_frame = np.full((50, 50), 128, dtype=np.uint8)
    
    for i in range(8):
        should_skip, motion_level = detector.detect_change(static_frame)
        process_frame = not should_skip  # Convert back to understand
        print(f"  Frame {i+1}: Motion={motion_level:.3f}, Should Skip={should_skip}, Process={process_frame}")
    
    print(f"\\nResetting detector...")
    detector.reset()
    
    # Test dynamic scene - all frames different
    print("\\nTesting dynamic scene (all frames random):")
    
    for i in range(8):
        dynamic_frame = np.random.randint(0, 255, (50, 50), dtype=np.uint8)
        should_skip, motion_level = detector.detect_change(dynamic_frame)
        process_frame = not should_skip  # Convert back to understand
        print(f"  Frame {i+1}: Motion={motion_level:.3f}, Should Skip={should_skip}, Process={process_frame}")
    
    print(f"\\nFinal detector state:")
    print(f"  static_frame_count: {detector.static_frame_count}")
    print(f"  dynamic_frame_count: {detector.dynamic_frame_count}")
    print(f"  current_state (is_static): {detector.current_state}")


if __name__ == "__main__":
    debug_scene_detection()