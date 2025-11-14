# QT to Raylib Migration for Cabana Tool

## Overview
This directory contains the initial work toward migrating the cabana tool from QT to Raylib to achieve complete QT removal from the sunnypilot project.

## Files Created

### raylib_poc/ Directory
- `cabana_raylib_main.cc` - New main entry point using Raylib instead of QApplication
- `cabana_ui_components.h/cc` - Base UI component architecture for Raylib
- `cabana_panels.h/cc` - Implementation of key UI panels (Messages, Video, Charts, Menu)
- `SConscript` - Build configuration for the Raylib implementation (standalone)

## Key Features Implemented

### 1. New UI Architecture
- Created a custom UI element base class following Raylib patterns
- Implemented layout management without QT docking system
- Created panel implementations for core cabana functionality:
  - Messages panel (with selection and scrolling)
  - Video panel (with playback controls)
  - Charts panel (with data visualization placeholder)
  - Menu bar (with file/edit/view/help structure)

### 2. QT Dependency Removal
- Created header replacements for QT-dependent components
- Removed QT includes from main entry point
- Implemented command-line parsing without QCommandLineParser

### 3. Integration Considerations
- Maintained compatibility with existing stream infrastructure (AbstractStream, ReplayStream, etc.)
- Preserved core cabana functionality while replacing UI layer

## Implementation Status

The proof-of-concept demonstrates:
- ✅ Basic Raylib-based application structure
- ✅ Custom UI component architecture
- ✅ Implementation of core UI panels
- ✅ Replacement of QT-dependent UI elements
- ✅ Command-line argument handling without QT

## Next Steps for Complete Migration

To fully migrate cabana from QT to Raylib, the following would need to be completed:

1. Complete implementation of all UI components (not just basic panels)
2. Integrate with existing cabana functionality (DBC management, signal analysis, etc.)
3. Update the SCons build system to use Raylib instead of QT
4. Replace all QT-dependent features (dialogs, file operations, threading, etc.)
5. Thorough testing and validation

## Notes

This represents a significant architectural change that would require extensive testing and validation. The implementation here serves as a foundation for the complete migration.

## Files Created
- raylib_poc/cabana_raylib_main.cc
- raylib_poc/cabana_ui_components.h
- raylib_poc/cabana_ui_components.cc
- raylib_poc/cabana_panels.h
- raylib_poc/cabana_panels.cc
- raylib_poc/SConscript
- Updated cabana.cc, mainwin.h
- Backups of original files (with .qt_backup extension)