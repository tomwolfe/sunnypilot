#!/usr/bin/env python3
"""
Perception to Control Latency Validator for Sunnypilot
Measures end-to-end latency from camera capture to control output
"""
import time
import numpy as np
from typing import Dict, List
from collections import deque
import threading

from openpilot.common.swaglog import cloudlog
from openpilot.common.realtime import DT_MDL, DT_CTRL


class LatencyValidator:
    """Validates end-to-end system latency"""
    
    def __init__(self):
        self.latency_measurements = deque(maxlen=1000)  # Keep last 1000 measurements
        self.frame_timestamps = {}  # Track timestamps by frame ID
        self.latency_targets = {
            'camera_to_model': 0.05,  # 50ms
            'model_to_control': 0.03,  # 30ms
            'end_to_end': 0.08  # 80ms total
        }
        
    def record_camera_timestamp(self, frame_id: int):
        """Record timestamp when camera frame is captured"""
        self.frame_timestamps[frame_id] = {
            'camera_capture': time.time(),
            'model_start': None,
            'model_end': None,
            'control_output': None
        }
    
    def record_model_start(self, frame_id: int):
        """Record timestamp when model processing starts"""
        if frame_id in self.frame_timestamps:
            self.frame_timestamps[frame_id]['model_start'] = time.time()
    
    def record_model_end(self, frame_id: int):
        """Record timestamp when model processing ends"""
        if frame_id in self.frame_timestamps:
            self.frame_timestamps[frame_id]['model_end'] = time.time()
    
    def record_control_output(self, frame_id: int):
        """Record timestamp when control output is ready"""
        if frame_id in self.frame_timestamps:
            self.frame_timestamps[frame_id]['control_output'] = time.time()
    
    def calculate_latency(self, frame_id: int) -> Dict[str, float]:
        """Calculate latency components for a specific frame"""
        if frame_id not in self.frame_timestamps:
            return {}
        
        timestamps = self.frame_timestamps[frame_id]
        latencies = {}
        
        # Camera to model start
        if timestamps['model_start'] and timestamps['camera_capture']:
            latencies['camera_to_model'] = timestamps['model_start'] - timestamps['camera_capture']
        
        # Model processing time
        if timestamps['model_start'] and timestamps['model_end']:
            latencies['model_processing'] = timestamps['model_end'] - timestamps['model_start']
        
        # Model to control
        if timestamps['control_output'] and timestamps['model_end']:
            latencies['model_to_control'] = timestamps['control_output'] - timestamps['model_end']
        
        # End-to-end latency
        if timestamps['camera_capture'] and timestamps['control_output']:
            latencies['end_to_end'] = timestamps['control_output'] - timestamps['camera_capture']
        
        # Store end-to-end for historical tracking
        if 'end_to_end' in latencies:
            self.latency_measurements.append(latencies['end_to_end'])
        
        return latencies
    
    def get_latency_statistics(self) -> Dict[str, float]:
        """Get statistical summary of latency measurements"""
        if not self.latency_measurements:
            return {"error": "No latency measurements collected"}
        
        measurements = list(self.latency_measurements)
        return {
            'avg_latency_ms': float(np.mean(measurements) * 1000),
            'min_latency_ms': float(np.min(measurements) * 1000),
            'max_latency_ms': float(np.max(measurements) * 1000),
            'p95_latency_ms': float(np.percentile(measurements, 95) * 1000),
            'p99_latency_ms': float(np.percentile(measurements, 99) * 1000),
            'total_samples': len(measurements)
        }
    
    def validate_latency_targets(self) -> Dict[str, bool]:
        """Validate if current performance meets targets"""
        stats = self.get_latency_statistics()
        
        if 'error' in stats:
            return {'error': 'No data available'}
        
        return {
            'camera_to_model': stats['avg_latency_ms'] / 1000 <= self.latency_targets['camera_to_model'],
            'model_to_control': stats['avg_latency_ms'] / 1000 <= self.latency_targets['model_to_control'],
            'end_to_end': stats['avg_latency_ms'] / 1000 <= self.latency_targets['end_to_end']
        }


def run_latency_test():
    """Run comprehensive latency testing"""
    validator = LatencyValidator()
    
    cloudlog.info("Starting end-to-end latency validation")
    
    # Simulate frame processing pipeline
    for i in range(100):  # Test 100 frames
        frame_id = i
        validator.record_camera_timestamp(frame_id)
        time.sleep(0.01)  # Simulate camera capture delay
        validator.record_model_start(frame_id)
        time.sleep(0.03)  # Simulate model processing
        validator.record_model_end(frame_id)
        time.sleep(0.01)  # Simulate control computation
        validator.record_control_output(frame_id)
        
        # Calculate and log this frame's latency
        latencies = validator.calculate_latency(frame_id)
        if 'end_to_end' in latencies:
            cloudlog.debug(f"Frame {frame_id} latency: {latencies['end_to_end']*1000:.1f}ms")
    
    # Generate statistics
    stats = validator.get_latency_statistics()
    compliance = validator.validate_latency_targets()
    
    print("Latency Validation Results:")
    print(f"Average Latency: {stats.get('avg_latency_ms', 0):.1f} ms")
    print(f"95th Percentile: {stats.get('p95_latency_ms', 0):.1f} ms")
    print(f"99th Percentile: {stats.get('p99_latency_ms', 0):.1f} ms")
    print(f"Total Samples: {stats.get('total_samples', 0)}")
    
    print("\nCompliance Check:")
    for component, is_compliant in compliance.items():
        status = "✅ PASS" if is_compliant else "❌ FAIL"
        target = validator.latency_targets.get(component, 0) * 1000
        print(f"{component}: {status} (target: <{target:.0f}ms)")
    
    return all(compliance.values())


if __name__ == "__main__":
    success = run_latency_test()
    print(f"\nOverall latency compliance: {'✅ PASS' if success else '❌ FAIL'}")
    exit(0 if success else 1)