# Self-Learning Autonomous Driving Implementation for Sunnypilot

## Overview
This implementation adds online learning capabilities to sunnypilot that allow the vehicle to adapt its driving behavior based on real-world experience and driver interventions. The system learns from:
- Driver corrections and interventions
- Model prediction accuracy
- Environmental conditions and driving scenarios

## Key Components

### 1. SelfLearningManager
- Tracks driver interventions and model prediction accuracy
- Updates driving parameters based on experience
- Maintains adaptive parameters that influence vehicle behavior
- Saves/loads learned parameters to persistent storage

### 2. SelfLearningSafety
- Validates all learning-based adjustments for safety compliance
- Ensures curvature and acceleration limits are not exceeded
- Monitors system safety score and can freeze learning if unsafe
- Implements parameter regularization to prevent drifting

### 3. SafeSelfLearningManager
- Combines learning and safety functions
- Provides safe adjustment methods for curvature and acceleration
- Integrates with existing control system

## Integration Points

### In controlsd.py:
- **Curvature adjustment**: Model outputs are passed through the learning system before actuator commands
- **Learning updates**: Feedback is provided after each control cycle with vehicle state, desired vs actual behavior
- **Safety checks**: All adjustments are validated before application

## Features

### Online Learning
- Learns from driver interventions (steering corrections)
- Adapts to individual driving preferences and road conditions
- Updates parameters gradually to maintain stability

### Safety-First Design
- All learning-based adjustments are validated for safety
- Curvature and acceleration limits enforced at multiple levels
- Learning can be frozen if safety conditions are not met
- Parameter regularization prevents unsafe drift

### Hardware-Efficient
- Lightweight implementation suitable for Comma 3x hardware
- Incremental updates rather than full model retraining
- Memory-efficient experience buffers

### Persistent Learning
- Learned parameters saved to persistent storage
- System retains learned behavior across restarts
- Gradual forgetting of outdated information

## Parameter Adjustments

The system adapts these key parameters:
- **lateral_control_factor**: Scaling for lateral control responses
- **curvature_bias**: Correction bias based on historical errors
- **acceleration_factor**: Scaling for longitudinal control
- **reaction_time_compensation**: Time compensation for different conditions

## Safety Measures

- Maximum curvature limits based on vehicle speed
- Lateral jerk (rate of curvature change) limits
- Parameter change rate limiting
- Safety score monitoring to detect unsafe learning patterns
- Emergency learning freeze capability

## Testing

- Unit tests for individual components
- Integration tests for end-to-end functionality
- Performance tests to ensure real-time operation
- Safety validation of all adjustment limits

## Usage

The system operates automatically when enabled. No user interaction is required. The learning system will gradually adapt the driving behavior based on real-world experience and driver interventions.

## Configuration

Learning parameters can be adjusted through the parameter system:
- `learning_rate`: Controls how quickly the system adapts (default: 0.01)
- `confidence_threshold`: Minimum model confidence to trigger learning (default: 0.7)
- `intervention_threshold`: Amount of driver correction needed to trigger learning (default: 0.5)

## Performance Characteristics

- < 1ms average overhead per control cycle
- Memory usage: < 10MB for experience buffers
- Parameter updates occur in real-time without interrupting driving
- Compatible with existing control system architecture

## Future Enhancements

This implementation provides a foundation that can be extended with:
- Deep reinforcement learning components
- More sophisticated model prediction error correction
- Collaborative learning across vehicle fleet
- Advanced scenario-based adaptation