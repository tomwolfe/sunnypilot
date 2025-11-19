# Integration Guide

This document describes how the different enhanced systems in sunnypilot integrate together:

## System Integration Architecture

The sunnypilot system follows a layered architecture with the following key integrations:

### 1. Perception Layer
- Enhanced vision processing integrates with ARM NEON optimizations
- Multi-camera fusion with validation metrics publisher
- Real-time object detection and tracking with YOLOv8
- Depth estimation and feature extraction modules

### 2. Planning Layer
- Advanced planning system with behavior prediction
- Predictive planning with scenario awareness
- Multi-layer fusion for enhanced accuracy
- Kalman filter-based tracking

### 3. Control Layer
- Adaptive control with dynamic adjustment
- Safety supervision and validation
- Thermal management integration
- Performance scaling based on conditions

### 4. Validation Layer
- Enhanced validation metrics publisher and consumer
- Real-time safety validation
- Situation-aware confidence scoring
- Redundant validation systems

### 5. UI Layer
- Raylib-based UI system
- Resource-efficient rendering
- Real-time system status displays
- Adaptive UI based on driving state

## Key Integration Points

### Enhanced Validation Integration
The enhanced validation system integrates with:
- Model outputs via the validation metrics publisher
- Perception system for confidence scoring
- Planning system for trajectory validation
- Safety supervisor for emergency detection

### ARM Optimization Integration
ARM NEON optimizations integrate with:
- Neural network inference for model efficiency
- Vision processing for real-time performance
- Mathematical operations for computational efficiency
- Memory management for allocation efficiency

### Thermal Management Integration
Thermal management integrates with:
- Performance scaling for thermal protection
- System monitoring for thermal status
- Resource allocation for thermal efficiency
- UI rendering for thermal-aware operation

### Data Flow
Data flows through the system as follows:
1. Raw sensor data → Perception layer → Enhanced validation
2. Enhanced validation → Planning layer → Control system
3. Control system → Safety supervisor → Vehicle actuators
4. System status → UI layer → Driver display
5. Performance metrics → Resource monitoring → Adaptive scaling

## API Integrations

### Messaging Interface
The system uses cereal messaging for all inter-process communication:
- Model outputs to validation system
- Validation results to planning system
- Planning outputs to control system
- System status to UI system

### Parameter Interface
Parameters are shared through the Params system:
- Configuration settings
- Calibration data
- Validation thresholds
- Performance targets

## Resource Management

The system implements efficient resource management:
- Memory pooling to reduce allocation overhead
- ARM-optimized operations for efficiency
- Thermal-based performance scaling
- Adaptive UI complexity based on system load