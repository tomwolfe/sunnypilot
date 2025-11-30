#!/usr/bin/env bash

set -e

echo "Fixing scipy installation on comma device..."

# Check if running on device
if [ ! -f /TICI ] && [ ! -f /EON ]; then
  echo "This script should be run on the comma device"
  exit 1
fi

# NOTE: This script expects system dependencies to already be installed
# by the main install.sh script. Running this script separately may require
# manually installing system dependencies first.

# Determine the correct pip to use based on whether a virtual environment is active
if [ -n "$VIRTUAL_ENV" ]; then
  echo "Virtual environment detected: $VIRTUAL_ENV"
  PYTHON_CMD="python"
  echo "Installing scipy in the active virtual environment..."

  # Upgrade pip in the virtual environment
  pip install --upgrade pip
else
  echo "No virtual environment detected, installing for system user..."
  PYTHON_CMD="python3"
  echo "Using --user flag for system-wide user installation..."

  # Upgrade pip for user
  python3 -m pip install --upgrade pip
fi

# Install or reinstall scipy with specific flags for ARM architecture
echo "Attempting to install scipy..."

# Try installing scipy directly first
if [ -n "$VIRTUAL_ENV" ]; then
  # If in virtual environment, install without --user flag
  if pip install --upgrade --force-reinstall scipy; then
    echo "scipy installed successfully in virtual environment"
  else
    echo "Direct scipy install failed in virtual environment, trying with no cache..."
    if pip install --no-cache-dir --force-reinstall scipy; then
      echo "scipy installed successfully with no cache in virtual environment"
    else
      echo "Regular install failed, trying with specific optimizations for ARM..."
      # Install with specific flags that are known to work on ARM platforms
      pip install --no-cache-dir --no-binary scipy --force-reinstall scipy
    fi
  fi
else
  # If not in virtual environment, use --user flag
  if python3 -m pip install --user --upgrade --force-reinstall scipy; then
    echo "scipy installed successfully with direct install"
  else
    echo "Direct scipy install failed, trying with no cache..."
    if python3 -m pip install --user --no-cache-dir --force-reinstall scipy; then
      echo "scipy installed successfully with no cache"
    else
      echo "Regular install failed, trying with specific optimizations for ARM..."
      # Install with specific flags that are known to work on ARM platforms
      python3 -m pip install --user --no-cache-dir --no-binary scipy --force-reinstall scipy
    fi
  fi
fi

# Verify installation
echo "Verifying scipy installation..."
if $PYTHON_CMD -c "import scipy; print(f'scipy version: {scipy.__version__}')"; then
  echo "scipy installation verified successfully!"
else
  echo "scipy installation failed or not accessible"
  exit 1
fi

echo "scipy installation fix completed!"