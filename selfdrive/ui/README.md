"""
Sunnypilot UI System - Documentation and Integration Guide

This document provides a comprehensive overview of the Sunnypilot UI System
designed for the Comma Three device with raylib rendering engine.

Architecture Overview
====================
The Sunnypilot UI System follows a layered architecture:

1. Data Integration Layer:
   - Connects to cereal messaging system
   - Integrates with perception, navigation, safety, and hardware systems
   - Provides real-time metrics and status updates

2. UI Component Layer:
   - Hardware Status Dashboard: CPU/RAM/Power usage visualization
   - Navigation Display: Route visualization and turn-by-turn guidance
   - Perception Visualization: Object detection and lane boundary display
   - System Status Panel: Safety validation and sensor health indicators
   - Controls Interface: Engage/disengage controls and settings access

3. Rendering Layer:
   - Optimized for ARM processors using raylib
   - Efficient drawing routines for low resource usage
   - Proper z-ordering for HUD overlay functionality

Performance Requirements
========================
- UI must consume <5% CPU when active
- Rendering must maintain 30+ FPS without affecting driving systems
- Memory usage under 50MB
- No input lag for critical controls

Safety Considerations
====================
- Critical safety information always visible
- Clear indication when autonomous system is engaged/disengaged
- Emergency status and intervention indicators
- Priority-based information hierarchy

File Structure
==============
selfdrive/ui/
├── sunnypilot_ui.py          # Main UI architecture
├── raylib_ui_system.py       # Raylib implementation
├── data_integration.py       # Real-time data integration
├── complete_ui_system.py     # Complete system integration
├── components/
│   ├── hardware_status.py    # Hardware status dashboard
│   ├── navigation_display.py # Navigation system
│   ├── perception_visualization.py # Perception visualization
│   ├── system_status.py      # System status panel
│   └── controls_interface.py # Controls interface
└── ui_state.py               # UI state management (existing)

Key Features
===========
1. Adaptive UI that changes based on driving context
2. Day/night theme support for various driving conditions
3. Resource-efficient rendering optimized for ARM processors
4. Real-time data visualization from all system components
5. Safety-first design with critical information prioritization
6. Emergency handling and intervention indicators
7. Performance monitoring and resource usage tracking

Integration Points
================
The UI system integrates with:
- cereal messaging (modelV2, controlsState, selfdriveState, deviceState, etc.)
- Advanced safety validation system
- Navigation system (navInstruction, navRoute)
- Hardware monitoring (deviceState, peripheralState)
- Perception system (modelV2, radarState)

Performance Benchmarks
====================
Target Performance:
- Frame Rate: 30 FPS minimum
- CPU Usage: <5% during normal operation
- Memory Usage: <50MB
- Render Time: <33ms per frame (30fps requirement)
- Data Update Frequency: Per message availability

Testing Requirements
==================
The system should be tested for:
1. Resource usage under various driving conditions
2. Safety critical information visibility
3. Response time for user inputs
4. Data accuracy and freshness
5. Failure mode handling
6. Emergency situation response

Deployment Notes
==============
- The UI system is designed for 1280x720 resolution (Comma Three)
- Optimized for ARM Cortex-A72 architecture
- Compatible with existing openpilot/sunnypilot messaging infrastructure
- Supports both development (desktop) and embedded (Comma Three) environments
"""