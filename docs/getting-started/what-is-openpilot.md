# What is sunnypilot?

[sunnypilot](http://github.com/sunnyhaibin/sunnypilot) is an enhanced open source driver assistance system. sunnypilot extends the capabilities of openpilot with advanced features including Adaptive Cruise Control (ACC), Automated Lane Centering (ALC), Forward Collision Warning (FCW), Lane Departure Warning (LDW), and additional capabilities like predictive planning, thermal management, and enhanced validation systems for a growing variety of [supported car makes, models, and model years](https://github.com/sunnyhaibin/sunnypilot/blob/master/docs/CARS.md). In addition, while sunnypilot is engaged, a camera-based Driver Monitoring (DM) feature alerts distracted and asleep drivers. See more about [the vehicle integration](https://github.com/sunnyhaibin/sunnypilot/blob/master/docs/INTEGRATION.md) and [limitations](https://github.com/sunnyhaibin/sunnypilot/blob/master/docs/LIMITATIONS.md).

## Enhanced Capabilities

sunnypilot includes advanced features that build on top of the base openpilot system:

* **Enhanced Validation and Safety**: Advanced validation metrics publisher and consumer system with situation-aware confidence scoring and safety checks
* **ARM NEON Optimization**: Specialized ARM NEON optimization for ARM64 processors for improved performance
* **Advanced UI Components**: New UI components for hardware status, navigation, perception visualization, and system status
* **Raylib UI System**: Efficient rendering system using Raylib optimized for ARM processors
* **Enhanced Vision Processing**: Improved depth estimation, feature extraction, and multi-camera fusion
* **Model Efficiency**: Optimizations including quantization and pruning for better performance
* **Performance Validation Tools**: Comprehensive benchmarking and validation tools
* **System Monitoring**: Real-time system health, thermal management, and resource monitoring
* **Predictive Planning**: Behavior prediction and scenario-aware planning
* **Safety Supervision**: Redundant safety validation and emergency response systems
* **Thermal Management**: Dynamic thermal-based performance scaling
* **YAML Configuration**: Flexible system configuration using YAML files
* **Enhanced Fusion**: Advanced multi-camera object tracking and fusion
* **Efficient Memory Management**: Memory pooling and optimization
* **Hardware Constraint Validation**: Specific validation for Comma Three platform requirements
* **Simulation Framework**: Comprehensive testing environment
* **Behavior Prediction**: Advanced object tracking and prediction using Kalman filters
* **State Management**: Robust lifecycle and state management for UI components


## How do I use it?

sunnypilot is designed to be used on the comma three and comma 3X.

## How does it work?

In short, sunnypilot uses the car's existing APIs for the built-in [ADAS](https://en.wikipedia.org/wiki/Advanced_driver-assistance_system) system and provides enhanced acceleration, braking, and steering inputs along with advanced safety validation, predictive planning, and resource optimization compared to the stock system.
