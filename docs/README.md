# openpilot docs

This is the source for [docs.sunnypilot.ai](https://docs.sunnypilot.ai).
The site is updated on pushes to master by this [workflow](../.github/workflows/docs.yaml).

## Features Overview

sunnypilot is an advanced open source driver assistance system with the following key capabilities:

### Core Features
- Driver assistance for 300+ supported car makes and models
- Adaptive Cruise Control (ACC)
- Lane Keeping Assist (LKA)
- Lane Change Assist (LCA)
- Traffic Jam Assist (TJA)
- Highway Assist (HWA)

### Enhanced Features in Latest Version
- **Enhanced Safety Validation**: Advanced validation metrics publisher and consumer system with situation-aware confidence scoring and safety checks
- **ARM NEON Optimization**: Specialized ARM NEON optimization for ARM64 processors for improved performance
- **Advanced UI Components**: New UI components for hardware status, navigation, perception visualization, and system status
- **Raylib UI System**: Efficient rendering system using Raylib optimized for ARM processors
- **Enhanced Vision Processing**: Improved depth estimation, feature extraction, and multi-camera fusion
- **Model Efficiency**: Optimizations including quantization and pruning for better performance
- **Performance Validation Tools**: Comprehensive benchmarking and validation tools
- **System Monitoring**: Real-time system health, thermal management, and resource monitoring
- **Predictive Planning**: Behavior prediction and scenario-aware planning
- **Safety Supervision**: Redundant safety validation and emergency response systems
- **Thermal Management**: Dynamic thermal-based performance scaling
- **YAML Configuration**: Flexible system configuration using YAML files
- **Enhanced Fusion**: Advanced multi-camera object tracking and fusion
- **Efficient Memory Management**: Memory pooling and optimization
- **Hardware Constraint Validation**: Specific validation for Comma Three platform requirements
- **Simulation Framework**: Comprehensive testing environment
- **Behavior Prediction**: Advanced object tracking and prediction using Kalman filters
- **State Management**: Robust lifecycle and state management for UI components

## Development
NOTE: Those commands must be run in the root directory of openpilot, **not /docs**

**1. Install the docs dependencies**
``` bash
pip install .[docs]
```

**2. Build the new site**
``` bash
mkdocs build
```

**3. Run the new site locally**
``` bash
mkdocs serve
```

References:
* https://www.mkdocs.org/getting-started/
* https://github.com/ntno/mkdocs-terminal
