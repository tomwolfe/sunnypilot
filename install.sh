#!/usr/bin/env bash

set -e

# Set DIR to the directory containing this script
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

echo "Starting sunnypilot installation..."

# Check if running on device
if [ -f /TICI ] || [ -f /EON ]; then
  echo "Running on comma device"
  
  # Install Python dependencies
  echo "Installing Python dependencies..."
  if [ -f "$DIR/requirements.txt" ]; then
    # Use the python environment's pip
    if command -v python3 &> /dev/null; then
      PYTHON_CMD=python3
    else
      PYTHON_CMD=python
    fi
    
    # Install dependencies from requirements.txt
    if $PYTHON_CMD -m pip install -r "$DIR/requirements.txt" --user; then
      echo "Python dependencies installed successfully"
    else
      echo "Warning: Failed to install some Python dependencies, continuing anyway..."
    fi
  else
    echo "No requirements.txt found, skipping dependency installation"
  fi

  # Install with uv as fallback (if available)
  if command -v uv &> /dev/null; then
    echo "Installing with uv..."
    uv sync --frozen --all-extras --locked || echo "uv installation failed, continuing..."
  fi

  # Run scons to build necessary components
  echo "Building system components..."
  if command -v scons &> /dev/null; then
    scons
  else
    echo "scons not available, skipping build"
  fi

  echo "Installation completed"
else
  echo "Not running on device, skipping device-specific installation"
fi