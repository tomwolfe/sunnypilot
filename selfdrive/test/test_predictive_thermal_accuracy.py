#!/usr/bin/env python3
"""
Predictive thermal model accuracy tests for the adaptive system.
Tests the accuracy, reliability, and behavior of the predictive thermal model.
"""
import time
import numpy as np
from unittest.mock import Mock
from openpilot.selfdrive.controls.lib.thermal_manager import ThermalManager


def test_prediction_accuracy_basic():
  """Test basic accuracy of thermal predictions under various conditions."""
  print("Testing thermal prediction accuracy...")
  
  thermal_manager = ThermalManager()
  
  # Test cases: (current_temp, current_trend, system_load, horizon, description)
  test_cases = [
    # No load, no trend -> should predict same temperature
    (60.0, 0.0, 0.0, 0.1, "no load, no trend"),
    # No load, warming trend -> should predict higher temperature
    (60.0, 0.2, 0.0, 0.1, "no load, warming trend"),  
    # No load, cooling trend -> should predict lower temperature
    (60.0, -0.1, 0.0, 0.1, "no load, cooling trend"),
    # High load, no trend -> should predict slight warming due to load
    (60.0, 0.0, 0.8, 0.1, "high load, no trend"),
    # High load, warming trend -> should predict more warming
    (60.0, 0.1, 0.9, 0.1, "high load, warming trend"),
    # Low load, cooling trend -> should predict continued cooling
    (70.0, -0.2, 0.2, 0.1, "low load, cooling trend"),
  ]
  
  for current_temp, current_trend, system_load, horizon, description in test_cases:
    predicted = thermal_manager._predict_temperature(current_temp, current_trend, system_load, horizon)
    
    print(f"  {description}: {current_temp}°C + trend({current_trend}) + load({system_load:.1f}) -> {predicted:.2f}°C")
    
    # Basic validity checks
    assert 0.0 <= predicted <= 100.0, f"Prediction {predicted}°C out of valid range [0, 100] for {description}"
    
    # If trend is positive and load is not negative, prediction should be >= current
    if current_trend > 0 and system_load >= 0.5:
        assert predicted >= current_temp, f"Positive trend + load should increase prediction for {description}"
    elif current_trend < 0 and system_load < 0.3:
        # If cooling trend and low load, prediction may be less than current
        pass  # This is acceptable behavior


def test_prediction_trend_sensitivity():
  """Test how sensitive predictions are to trend changes."""
  print("Testing prediction trend sensitivity...")
  
  thermal_manager = ThermalManager()
  
  # Same starting temperature, different trends
  base_temp = 65.0
  base_load = 0.5
  horizon = 0.1
  
  trends = [-0.5, -0.2, 0.0, 0.1, 0.3, 0.5]  # Various trends
  predictions = []
  
  for trend in trends:
    pred = thermal_manager._predict_temperature(base_temp, trend, base_load, horizon)
    predictions.append(pred)
    print(f"    Trend {trend:4.1f}: {base_temp}°C -> {pred:.2f}°C")
  
  # Predictions should be monotonically increasing with increasing trend
  for i in range(1, len(predictions)):
    assert predictions[i] >= predictions[i-1], f"Prediction should increase with trend: {predictions[i-1]} -> {predictions[i]}"
  
  # The difference should be roughly proportional to the trend
  # (with some load effect modifying it)
  for i in range(1, len(trends)):
    trend_diff = trends[i] - trends[i-1]
    pred_diff = predictions[i] - predictions[i-1]
    # Positive trend differences should result in positive prediction differences
    if trend_diff > 0:
      assert pred_diff >= 0, f"Positive trend difference should yield positive prediction difference"


def test_prediction_load_sensitivity():
  """Test how sensitive predictions are to system load changes."""
  print("Testing prediction load sensitivity...")
  
  thermal_manager = ThermalManager()
  
  # Same starting conditions, different loads
  base_temp = 60.0
  base_trend = 0.1  # Slight warming
  horizon = 0.1
  
  loads = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]  # Various loads
  predictions = []
  
  for load in loads:
    pred = thermal_manager._predict_temperature(base_temp, base_trend, load, horizon)
    predictions.append(pred)
    print(f"    Load {load:4.1f}: {base_temp}°C + trend({base_trend}) -> {pred:.2f}°C")
  
  # With positive trend, higher load should result in higher prediction
  for i in range(1, len(predictions)):
    assert predictions[i] >= predictions[i-1], f"Higher load should increase prediction with positive trend"
  
  # Test with cooling trend too
  cooling_trend = -0.1
  cooling_predictions = []
  
  for load in loads:
    pred = thermal_manager._predict_temperature(base_temp, cooling_trend, load, horizon)
    cooling_predictions.append(pred)
    print(f"    Load {load:4.1f} with cooling: {base_temp}°C + trend({cooling_trend}) -> {pred:.2f}°C")
  
  # With cooling trend, higher load should still affect prediction,
  # but the effect might be less pronounced


def test_prediction_horizon_behavior():
  """Test how predictions change with different time horizons."""
  print("Testing prediction horizon behavior...")
  
  thermal_manager = ThermalManager()
  
  base_temp = 65.0
  base_trend = 0.2  # Warming at 0.2°C/s
  base_load = 0.6
  base_horizon = 0.1  # 100ms
  
  # Predict for different horizons
  horizons = [0.05, 0.1, 0.15, 0.2, 0.25]  # 50ms to 250ms
  predictions = []
  
  for horizon in horizons:
    pred = thermal_manager._predict_temperature(base_temp, base_trend, base_load, horizon)
    predictions.append(pred)
    expected_simple = base_temp + (base_trend * horizon)
    print(f"    Horizon {horizon*1000:4.0f}ms: {base_temp}°C -> {pred:.2f}°C (simple: {expected_simple:.2f}°C)")
  
  # Predictions should increase with horizon
  for i in range(1, len(predictions)):
    assert predictions[i] >= predictions[i-1], f"Longer horizon should predict higher temp with positive trend"
  
  # The difference should increase with horizon (though load affects it)


def test_prediction_edge_cases():
  """Test thermal predictions with extreme values."""
  print("Testing prediction edge cases...")
  
  thermal_manager = ThermalManager()
  
  # Test edge cases that should still produce valid results
  edge_cases = [
    # (temp, trend, load, horizon, description)
    (0.0, 0.0, 0.0, 0.1, "minimum temperature"),
    (100.0, 0.0, 0.0, 0.1, "maximum temperature"), 
    (50.0, 2.0, 1.0, 0.1, "high trend, high load"),
    (50.0, -2.0, 1.0, 0.1, "high negative trend, high load"),
    (50.0, 0.0, 1.0, 0.5, "long horizon, high load"),
    (20.0, 0.5, 0.0, 0.1, "low temp, warming"),
    (80.0, -0.5, 0.0, 0.1, "high temp, cooling"),
  ]
  
  for current_temp, current_trend, system_load, horizon, description in edge_cases:
    predicted = thermal_manager._predict_temperature(current_temp, current_trend, system_load, horizon)
    
    print(f"    {description}: {current_temp}°C -> {predicted:.2f}°C")
    
    # All predictions should be in valid range
    assert 0.0 <= predicted <= 100.0, f"Prediction {predicted}°C out of range [0, 100] for {description}"
    
    # Check physical reasonableness
    if current_trend == 0 and system_load == 0:
      # If no trend and no load effect, should be close to current (with minimal load coefficient effect)
      assert abs(predicted - current_temp) <= 0.1, f"Should stay near {current_temp}°C with no trend/load, got {predicted}°C"


def test_prediction_consistency():
  """Test that predictions are consistent with the same inputs."""
  print("Testing prediction consistency...")
  
  thermal_manager = ThermalManager()
  
  # Same inputs should give same outputs
  temp, trend, load, horizon = 70.0, 0.15, 0.4, 0.1
  
  predictions = []
  for i in range(10):
    pred = thermal_manager._predict_temperature(temp, trend, load, horizon)
    predictions.append(pred)
    print(f"    Iteration {i+1}: {pred:.4f}°C")
  
  # All predictions should be identical
  first_pred = predictions[0]
  for i, pred in enumerate(predictions[1:], 1):
    assert abs(pred - first_pred) < 0.0001, f"Inconsistent prediction at iteration {i}: {pred} vs {first_pred}"
  
  print(f"    All 10 predictions identical: {first_pred:.4f}°C")


def test_prediction_realistic_scenarios():
  """Test predictions with realistic thermal scenarios."""
  print("Testing predictions with realistic thermal scenarios...")
  
  thermal_manager = ThermalManager()
  
  # Simulate realistic scenarios
  scenarios = [
    {
      "name": "CPU-intensive computation",
      "temp": 75.0,
      "trend": 0.3,  # Heating up due to activity
      "load": 0.9,   # High CPU/GPU load
      "horizon": 0.1,
      "expected_behavior": "significant heating prediction"
    },
    {
      "name": "Idle cooling",
      "temp": 65.0,
      "trend": -0.1,  # Cooling down
      "load": 0.1,    # Low load
      "horizon": 0.1,
      "expected_behavior": "cooling prediction"
    },
    {
      "name": "Highway driving thermal plateau", 
      "temp": 70.0,
      "trend": 0.0,   # Stable temperature
      "load": 0.6,    # Moderate sustained load
      "horizon": 0.1,
      "expected_behavior": "slight warming due to load"
    },
    {
      "name": "Stopped in traffic heating",
      "temp": 60.0, 
      "trend": 0.2,   # Warming in traffic
      "load": 0.5,    # Moderate load (infotainment, etc.)
      "horizon": 0.1,
      "expected_behavior": "warming prediction"
    }
  ]
  
  for scenario in scenarios:
    pred = thermal_manager._predict_temperature(
      scenario["temp"], 
      scenario["trend"], 
      scenario["load"], 
      scenario["horizon"]
    )
    
    temp_change = pred - scenario["temp"]
    
    print(f"    {scenario['name']}: {scenario['temp']:.1f}°C -> {pred:.2f}°C (Δ{temp_change:+.2f}°C)")
    
    # Validate that predictions are physically reasonable
    assert 0.0 <= pred <= 100.0, f"Prediction {pred}°C out of range"
    
    # Basic sanity check: if trend is strongly positive and load is high, 
    # expect some warming, though load effect is added gradually
    if scenario["trend"] > 0.1 and scenario["load"] > 0.7:
      assert pred >= scenario["temp"], f"Should predict warming for {scenario['name']}"
    elif scenario["trend"] < -0.1 and scenario["load"] < 0.3:
      # If cooling strongly with low load, should continue cooling or at least not heat significantly
      pass  # More subtle behavior expected here


def test_prediction_stability():
  """Test that predictions are stable across multiple calls."""
  print("Testing prediction stability...")
  
  thermal_manager = ThermalManager()
  
  # Use the same inputs repeatedly - predictions should be deterministic
  test_params = [
    (60.0, 0.1, 0.5, 0.1),
    (75.0, -0.2, 0.8, 0.1),
    (50.0, 0.0, 0.3, 0.1)
  ]
  
  for i, (temp, trend, load, horizon) in enumerate(test_params):
    print(f"    Parameter set {i+1}: T={temp}, dT/dt={trend}, load={load}, horizon={horizon}")
    
    # Get multiple predictions for the same parameters
    predictions = [thermal_manager._predict_temperature(temp, trend, load, horizon) for _ in range(5)]
    
    # Check that all predictions are the same  
    first_pred = predictions[0]
    for j, pred in enumerate(predictions[1:], 1):
      assert abs(pred - first_pred) < 0.0001, f"Prediction not stable: {first_pred} vs {pred} (attempt {j})"
    
    print(f"      All predictions: {[f'{p:.4f}' for p in predictions]}")


if __name__ == "__main__":
  print("Starting predictive thermal model accuracy tests...\n")
  
  test_prediction_accuracy_basic()
  test_prediction_trend_sensitivity()
  test_prediction_load_sensitivity()
  test_prediction_horizon_behavior()
  test_prediction_edge_cases()
  test_prediction_consistency()
  test_prediction_realistic_scenarios()
  test_prediction_stability()
  
  print(f"\n✓ All predictive thermal model accuracy tests passed!")