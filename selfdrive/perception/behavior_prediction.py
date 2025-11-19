"""
Simple Behavior Prediction for Sunnypilot
Minimal behavior prediction with basic functionality
"""
import numpy as np
from typing import Dict, List, Tuple
from dataclasses import dataclass


@dataclass
class PredictedObject:
  """Simple predicted object"""
  position: np.ndarray  # [x, y, z]
  velocity: np.ndarray  # [vx, vy, vz] 
  acceleration: np.ndarray  # [ax, ay, az]
  predicted_trajectory: List[np.ndarray]  # List of future positions
  confidence: float


class SimpleBehaviorPredictor:
  """Simple behavior predictor with basic prediction"""
  
  def __init__(self, prediction_horizon: float = 3.0, dt: float = 0.1):
    self.prediction_horizon = prediction_horizon
    self.dt = dt
    self.time_steps = int(prediction_horizon / dt)
  
  def predict_object_trajectory(self, position: np.ndarray, velocity: np.ndarray, 
                               acceleration: np.ndarray = None) -> PredictedObject:
    """Predict trajectory for a single object"""
    if acceleration is None:
      acceleration = np.array([0.0, 0.0, 0.0])
    
    trajectory = []
    current_pos = position.copy()
    current_vel = velocity.copy()
    
    for _ in range(self.time_steps):
      # Simple physics integration: pos = pos + vel*dt
      current_pos = current_pos + current_vel * self.dt
      current_vel = current_vel + acceleration * self.dt
      trajectory.append(current_pos.copy())
    
    return PredictedObject(
      position=position,
      velocity=velocity,
      acceleration=acceleration,
      predicted_trajectory=trajectory,
      confidence=0.8  # Default confidence
    )
  
  def predict_multiple_objects(self, objects_data: List[Dict]) -> List[PredictedObject]:
    """Predict trajectories for multiple objects"""
    predictions = []
    
    for obj_data in objects_data:
      pos = np.array(obj_data.get('position', [0, 0, 0]))
      vel = np.array(obj_data.get('velocity', [0, 0, 0]))
      acc = np.array(obj_data.get('acceleration', [0, 0, 0]))
      
      prediction = self.predict_object_trajectory(pos, vel, acc)
      predictions.append(prediction)
    
    return predictions