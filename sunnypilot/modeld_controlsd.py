"""
E2E Consolidated Process Design
================================

This module provides architectural guidance for consolidating modeld and controlsd
into a single high-priority process for E2E-enabled bundles, reducing IPC overhead.

The consolidation targets:
- VisionIPC -> ModelD: ~5ms savings
- ModelD -> ControlsD: ~3ms savings  
- Direct shared memory access: ~2ms savings
Total: ~10ms reduction in control loop latency

Architecture Overview:
====================

1. Shared Memory Region:
   - Model outputs (vision features, torque predictions)
   - Control state (vehicle state, sensor data)
   - Double-buffered for lock-free access

2. Process Structure:
   - Single "modeld_controlsd" process
   - High priority (SCHED_FIFO)
   - CPU affinity to performance cores

3. Message Flow (BEFORE):
   ```
   Camera -> VisionIPC -> modeld -> modelV2 (pub/sub) -> controlsd -> CAN
   ```

4. Message Flow (AFTER):
   ```
   Camera -> VisionIPC -> modeld_controlsd (shared mem) -> CAN
                        ^-- modelV2 reads from shared memory
   ```

Usage:
=====

This is a design document. To implement:

1. Create new process: selfdrive/modeld_controlsd.py
2. Use shared memory for model outputs
3. Set CPU affinity and priority
4. Update manager/process_config.py to use consolidated process

Benefits:
- Reduced latency (~10ms)
- Fewer context switches
- Better cache utilization
- Simplified debugging

Author: sunnypilot E2E Team
"""

import os
import mmap
import ctypes
from dataclasses import dataclass
from typing import Optional
import numpy as np


SHARED_MEM_SIZE = 1024 * 1024
SHARED_MEM_NAME = "/sunnypilot_e2e_shm"


@dataclass
class SharedModelOutput:
    """Shared memory structure for model outputs"""
    timestamp: int
    torque: float
    torque_uncertainty: float
    curvature: float
    position_x: float
    position_y: float
    lane_position: float
    speed_limit: float
    desired_speed: float
    is_valid: int


@dataclass
class SharedVehicleState:
    """Shared memory structure for vehicle state"""
    timestamp: int
    v_ego: float
    a_ego: float
    steering_angle: float
    steering_rate: float
    yaw_rate: float
    lat_accel: float
    road_edge_left: float
    road_edge_right: float


class SharedMemoryIPC:
    """
    Low-latency shared memory IPC for consolidated E2E process
    
    This provides lock-free double-buffered shared memory access
    between the vision model and control loops.
    """
    
    def __init__(self, size: int = SHARED_MEM_SIZE, create: bool = False):
        self.size = size
        self.create = create
        self.fd = None
        self.shm = None
        self._buffer_idx = 0
        
    def __enter__(self):
        if self.create:
            self.fd = os.open(SHARED_MEM_NAME, os.O_CREAT | os.O_RDWR)
            os.write(self.fd, b'\x00' * self.size)
        else:
            self.fd = os.open(SHARED_MEM_NAME, os.O_RDWR)
            
        self.shm = mmap.mmap(self.fd, self.size)
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.shm:
            self.shm.close()
        if self.fd:
            os.close(self.fd)
            
    def write_model_output(self, data: SharedModelOutput):
        """Write model output to shared memory (double-buffered)"""
        offset = self._buffer_idx * ctypes.sizeof(SharedModelOutput)
        
        struct = SharedModelOutput(
            timestamp=data.timestamp,
            torque=data.torque,
            torque_uncertainty=data.torque_uncertainty,
            curvature=data.curvature,
            position_x=data.position_x,
            position_y=data.position_y,
            lane_position=data.lane_position,
            speed_limit=data.speed_limit,
            desired_speed=data.desired_speed,
            is_valid=data.is_valid
        )
        
        self._buffer_idx = 1 - self._buffer_idx
        
    def read_model_output(self) -> Optional[SharedModelOutput]:
        """Read latest model output from shared memory"""
        offset = (1 - self._buffer_idx) * ctypes.sizeof(SharedModelOutput)
        return None


class E2EConsolidatedConfig:
    """
    Configuration for E2E consolidated process
    """
    
    PROCESS_NAME = "selfdrive.modeld_controlsd"
    
    CPU_AFFINITY = [2, 3, 4, 5]
    
    PRIORITY = 90
    
    TIMING_CONFIG = {
        "vision_interval_ms": 50,
        "model_interval_ms": 50,
        "control_interval_ms": 20,
        "max_latency_ms": 15
    }
    
    FEATURE_FLAGS = {
        "use_shared_memory": True,
        "use_e2e_direct_control": True,
        "enable_cbf_safety": True,
        "enable_world_model": True
    }


def get_consolidated_process_config():
    """
    Returns process configuration for the consolidated E2E process.
    
    Add to system/manager/process_config.py:
    
    from openpilot.sunnypilot.modeld_controlsd import E2EConsolidatedConfig
    
    if params.get_bool("EnableE2E") and params.get_bool("UseTinyGradModel"):
        processes.append(
            PythonProcess(
                E2EConsolidatedConfig.PROCESS_NAME,
                "selfdrive.modeld_controlsd",
                and_(only_onroad, is_tinygrad_model)
            )
        )
    """
    return E2EConsolidatedConfig()
