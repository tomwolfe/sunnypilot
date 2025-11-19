# sunnypilot docs

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
- **Enhanced Safety Validation**: Advanced validation metrics with situation-aware confidence scoring
- **ARM NEON Optimization**: Specialized ARM NEON optimization for ARM64 processors
- **Advanced UI Components**: New UI components for system status and visualization
- **Enhanced Vision Processing**: Improved depth estimation and multi-camera fusion
- **Model Efficiency**: Optimizations including quantization and pruning
- **Performance Validation**: Comprehensive benchmarking and validation tools
- **System Monitoring**: Real-time health, thermal management, and resource monitoring
- **Predictive Planning**: Behavior prediction and scenario-aware planning
- **Safety Supervision**: Redundant safety validation and emergency response systems
- **Thermal Management**: Dynamic thermal-based performance scaling
- **Hardware Constraint Validation**: Validation for Comma Three platform requirements

## Development
NOTE: Those commands must be run in the root directory of sunnypilot, **not /docs**

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
