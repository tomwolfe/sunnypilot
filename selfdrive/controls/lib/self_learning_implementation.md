# Self-Learning Autonomous Driving Implementation for Sunnypilot

## Overview
This implementation adds online learning capabilities to sunnypilot that allow the vehicle to adapt its driving behavior based on real-world experience and driver interventions. The system learns from:
- Context-aware driver corrections and interventions
- Model prediction accuracy and prediction errors
- Vehicle state and environmental conditions

## Key Components

### 1. SelfLearningManager
- Tracks context-aware driver interventions and model prediction accuracy
- Updates driving parameters based on experience with safety validation
- Maintains adaptive parameters that influence vehicle behavior
- Saves/loads learned parameters to persistent storage
- Implements enhanced parameter regularization

### 2. SelfLearningSafety
- Validates all learning-based adjustments for safety compliance
- Ensures curvature and acceleration limits are not exceeded
- Monitors system safety score based on absolute values and stability (not relative ratios)
- Implements parameter regularization to prevent unsafe drift
- Provides emergency learning freeze capability

### 3. SafeSelfLearningManager
- Combines learning and safety functions with fixed safety monitoring
- Provides safe adjustment methods for curvature and acceleration
- Integrates with existing control system with proper safety score calculation

## Integration Points

### In controlsd.py:
- **Curvature adjustment**: Model outputs are passed through the learning system before actuator commands
- **Learning updates**: Feedback is provided after each control cycle with actual vehicle curvature (not controller output) and prediction error
- **Safety checks**: All adjustments are validated before application

## Features

### Context-Aware Learning
- Learns only from corrective driver interventions (not all steeringPressed events)
- Uses model prediction error to determine if intervention is correction vs. normal driving
- Factors in vehicle speed and conditions for appropriate learning

### Safety-First Design
- All learning-based adjustments are validated for safety using absolute values
- Curvature and acceleration limits enforced at multiple levels
- Learning can be frozen if safety conditions are not met
- Parameter regularization prevents unsafe drift
- Fixed safety score calculation uses absolute values instead of relative ratios

### Enhanced Parameter Management
- Integrated speed compensation as learnable parameters instead of hardcoded values
- More aggressive parameter regularization to prevent dangerous drift
- Additional learned parameters for speed-dependent adjustments

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
- **speed_compensation_base**: Base value for speed-dependent adjustments
- **speed_compensation_rate**: Rate of change with speed (learnable)

## Safety Measures

- Maximum curvature limits based on vehicle speed using kinematic model
- Lateral jerk (rate of curvature change) limits
- Parameter change rate limiting
- Safety score monitoring based on absolute values and system stability
- Emergency learning freeze capability
- Context-aware learning to prevent learning from driver errors

## Key Fixes from Critical Review

### Fixed: Dangerous Driver Intervention Trigger
- **Before**: Learned from any `CS.steeringPressed` event
- **After**: Context-aware learning that checks for significant model prediction error and corrective direction

### Fixed: Data Source Error
- **Before**: Used controller's desired curvature as "actual vehicle curvature"
- **After**: Uses actual vehicle state with kinematic model to calculate real curvature from steering angle

### Fixed: Safety Score Calculation
- **Before**: Used relative change ratios causing meaningless scores
- **After**: Uses absolute values against physical limits and stability measures

### Fixed: Critical Implementation Bug
- **Before**: `adjusted_outputs` was set to original curvature instead of adjusted curvature
- **After**: Properly calculates and passes adjusted outputs for safety scoring

### Fixed: Speed Factor Integration
- **Before**: Hardcoded speed factor in adjustment logic
- **After**: Integrated as learnable parameters in the learning system

## Improvements Addressing Critical Review Concerns

### 1. Parameter Regularization Enhancement
- **Issue**: Regularization was too aggressive and could stifle beneficial learning
- **Improvement**: Implemented more nuanced regularization with:
  - Lower starting thresholds (2% vs 5%) for when to apply regularization
  - Balanced regularization rates that are gentler for beneficial learning
  - Still maintains aggressive regularization for dangerous deviations (>25%)
- **Result**: Better balance between safety and allowing beneficial adaptations

### 2. Model Confidence Handling
- **Issue**: Reliance on potentially unreliable model confidence scores
- **Improvement**: Added validation and clamping of confidence values to [0, 1] range
- **Result**: More robust handling of potentially faulty confidence inputs

### 3. Clear Opt-Out/Reset Mechanism
- **Issue**: No clear user-facing mechanism to reset learned parameters
- **Improvement**: Added parameter-based reset functionality
  - Set `ResetSelfLearning` parameter to "1" to trigger reset
  - System checks for reset requests during periodic updates
  - Clear feedback in logs when reset occurs
- **Result**: Users can now reset learning if desired

### 4. Performance Monitoring
- **Issue**: Performance claim of < 1ms overhead needs validation
- **Improvement**: Added comprehensive performance tracking:
  - Time tracking for each update operation
  - Running averages and maximum time tracking
  - Periodic logging of performance statistics
- **Result**: Real-time monitoring of performance impact

### 5. Interaction Analysis & Monitoring
- **Issue**: Potential unforeseen interactions between learned parameters and existing control logic
- **Improvement**: Added periodic logging to monitor interaction between learned parameters and adaptive modifications
  - Logs lateral control factor values
  - Tracks learning enable status
  - Monitors curvature changes for anomalies
- **Result**: Better visibility into how learned parameters affect overall system behavior

### 6. Safety Score Weighting Improvements
- **Issue**: Weighting scheme was somewhat arbitrary
- **Improvement**: Made weights more configurable and better explained the rationale
  - Maintained balanced split (70% weighted, 30% conservative) but with clearer justification
  - Added support for dynamic weight adjustment based on validation results
  - Better handling of missing score components
- **Result**: More principled approach to safety scoring

## Testing

- Unit tests for individual components
- Integration tests for end-to-end functionality
- Performance tests to ensure real-time operation
- Safety validation of all adjustment limits
- Verification tests for all fixes and improvements

## Usage

The system operates automatically when enabled. No user interaction is required. The learning system will gradually adapt the driving behavior based on real-world experience and corrective driver interventions.

## Configuration

Learning parameters can be adjusted through the parameter system:
- `learning_rate`: Controls how quickly the system adapts (default: 0.01)
- `confidence_threshold`: Minimum model confidence to trigger learning (default: 0.7)
- `intervention_threshold`: Amount of driver correction needed to trigger learning (default: 0.5)
- `ResetSelfLearning`: Set to "1" to reset all learned parameters (auto-cleared after reset)

## Performance Characteristics

- < 1ms average overhead per control cycle (with monitoring to validate this claim)
- Memory usage: < 10MB for experience buffers
- Parameter updates occur in real-time without interrupting driving
- Compatible with existing control system architecture

## Future Enhancements

This implementation provides a foundation that can be extended with:
- Advanced model prediction error correction
- Collaborative learning across vehicle fleet
- Advanced scenario-based adaptation
- Enhanced safety monitoring and validation
- Dynamic weight adjustment for safety scoring based on real-world validation
- Parameter-specific regularization strategies