# Sunnypilot Settings Guide

This guide documents all the new settings and parameters added to sunnypilot, providing explanations of what each setting does and how to use them effectively.

## Cruise Control Settings

### Cruise Stand Still
- **Parameter**: `CruiseStandStill`
- **Type**: Boolean
- **Description**: Enable to allow cruise to come to a complete stop and resume automatically. Disable to use stock cruise.

### Smart Cruise Control
- **Parameter**: `SmartCruiseEnabled`
- **Type**: Boolean
- **Description**: Enable sunnypilot's smart cruise that automatically adjusts to traffic. Works with stock ACC.

### Cruise Speed Limit Offset
- **Parameter**: `CruiseSpeedLimitOffset`
- **Type**: Integer
- **Description**: Set the offset in mph/km/h from the speed limit that you are comfortable with.
- **Default Value**: 0

## Display Settings

### Display Always On
- **Parameter**: `DisplayAlwaysOn`
- **Type**: Boolean
- **Description**: Keep the display on at all times while the car is running.

### Auto-Brightness
- **Parameter**: `AutoBrightness`
- **Type**: Boolean
- **Description**: Automatically adjust screen brightness based on ambient light conditions.

### Display Timeout
- **Parameter**: `DisplayTimeout`
- **Type**: Integer
- **Range**: 5-60 seconds
- **Default Value**: 10
- **Description**: Set the timeout in seconds before the display turns off with Auto-Brightness disabled.

### UI Scale
- **Parameter**: `UIScale`
- **Type**: Integer
- **Range**: 70-130%
- **Default Value**: 100
- **Description**: Adjust the UI scale percentage (70-130%).

## Steering Settings

### LKAS Toggle
- **Parameter**: `LKASEnabled`
- **Type**: Boolean
- **Description**: Enable to turn on LKAS with a single tap on the gas pedal. Disable to use stock LKAS.

### LKAS Start Delay
- **Parameter**: `LKASStartDelay`
- **Type**: Integer
- **Range**: 0-10 seconds
- **Default Value**: 2
- **Description**: Set the delay in seconds before LKAS engages after a tap on the gas pedal.

### Auto Lane Change
- **Parameter**: `AutoLaneChangeEnabled`
- **Type**: Boolean
- **Description**: Enable to use sunnypilot's improved lane change assistant. No more need to hold blinker!

## OSM (OpenStreetMap) Integration Settings

### OSM Integration
- **Parameter**: `OSMEnabled`
- **Type**: Boolean
- **Description**: Enable OpenStreetMap integration for enhanced navigation and speed limit data.

### Speed Limit Sign Recognition
- **Parameter**: `SLSREnabled`
- **Type**: Boolean
- **Description**: Enable to use camera-based speed limit sign recognition in addition to map data.

### Map Scale to Speed
- **Parameter**: `MapScaleToSpeed`
- **Type**: Boolean
- **Description**: Automatically adjust map scale based on vehicle speed for better visibility.

## Visuals Settings

### Show Speed
- **Parameter**: `ShowSpeed`
- **Type**: Boolean
- **Description**: Display current speed in the onroad UI. Toggle to show or hide.

### Show Speed Limit
- **Parameter**: `ShowSpeedLimit`
- **Type**: Boolean
- **Description**: Display speed limit on the road. Adjusts based on map data and speed limit signs.

### Show ETA
- **Parameter**: `ShowETA`
- **Type**: Boolean
- **Description**: Show estimated time of arrival on the navigation screen.

### Brightness
- **Parameter**: `Brightness`
- **Type**: Integer
- **Range**: 50-100%
- **Default Value**: 75
- **Description**: Adjust display brightness percentage.

## Trips Settings

### Trip Logging
- **Parameter**: `TripLogging`
- **Type**: Boolean
- **Description**: Enable to record trip data including route, speed, and driving metrics.

### Fuel Efficiency Tracking
- **Parameter**: `FuelEfficiencyTracking`
- **Type**: Boolean
- **Description**: Track fuel efficiency and environmental impact metrics for your trips.

## Vehicle Settings

### Custom Vehicle Model
- **Parameter**: `CustomVehicleModel`
- **Type**: Boolean
- **Description**: Enable to use custom vehicle model parameters for improved control.

### Torque Vectoring
- **Parameter**: `TorqueVectoring`
- **Type**: Boolean
- **Description**: Enable advanced torque vectoring for better cornering and stability.

## How to Use UI Controls

### Option Control (OptionControlSP)
This control allows you to select numeric values within a range using +/- buttons:
- Use the "-" button to decrease the value
- Use the "+" button to increase the value
- The control respects the minimum and maximum values set for each parameter

### Input Dialog (InputDialogSP)
This dialog allows you to enter text values:
- Use the on-screen keyboard to enter values
- The dialog supports validation to ensure values are within expected ranges