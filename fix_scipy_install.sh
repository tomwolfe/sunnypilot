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

# Function to check if system dependencies for scipy are available
check_system_deps() {
  echo "Checking for required system dependencies..."

  # Check for gfortran
  if ! command -v gfortran &> /dev/null; then
    echo "WARNING: gfortran not found. scipy compilation may fail."
    echo "Please run install.sh first to install system dependencies."
    return 1
  fi

  # Check for libopenblas-dev (using dpkg on Debian-based systems like comma devices)
  if ! dpkg -l | grep -q libopenblas-dev; then
    echo "WARNING: libopenblas-dev not found. scipy compilation may fail."
    echo "Please run install.sh first to install system dependencies."
    return 1
  fi

  # Check for basic build tools
  if ! command -v gcc &> /dev/null || ! command -v g++ &> /dev/null; then
    echo "WARNING: gcc/g++ not found. scipy compilation may fail."
    echo "Please run install.sh first to install system dependencies."
    return 1
  fi

  echo "System dependencies check passed."
  return 0
}

# Try installing scipy and numpy together (scipy requires numpy)
if [ -n "$VIRTUAL_ENV" ]; then
  # If in virtual environment, install without --user flag
  if pip install --only-binary=numpy,scipy numpy scipy; then
    echo "numpy and scipy installed successfully with binary wheels in virtual environment"
  else
    echo "Binary install failed in virtual environment, trying with no cache..."
    if pip install --no-cache-dir --force-reinstall numpy scipy; then
      echo "numpy and scipy installed successfully with no cache in virtual environment"
    else
      echo "Regular install failed, checking system dependencies before trying source compilation..."
      if check_system_deps; then
        echo "System dependencies verified. Attempting source compilation (this may take 20-60 minutes for both numpy and scipy)..."
        echo "  - This is normal behavior for ARM devices"
        echo "  - Compiling scipy from source (with numpy) requires significant resources"
        echo "  - Please be patient and do not interrupt the process"
        # Install with specific flags that are known to work on ARM platforms
        # Install numpy first, then scipy
        pip install --no-cache-dir --no-binary numpy --force-reinstall numpy
        pip install --no-cache-dir --no-binary scipy --force-reinstall scipy
      else
        echo "System dependencies check failed. Please install system dependencies first."
        exit 1
      fi
    fi
  fi
else
  # If not in virtual environment, use --user flag
  if python3 -m pip install --user --only-binary=numpy,scipy numpy scipy; then
    echo "numpy and scipy installed successfully with binary wheels with direct install"
  else
    echo "Binary install failed, trying with no cache..."
    if python3 -m pip install --user --no-cache-dir --force-reinstall numpy scipy; then
      echo "numpy and scipy installed successfully with no cache"
    else
      echo "Regular install failed, checking system dependencies before trying source compilation..."
      if check_system_deps; then
        echo "System dependencies verified. Attempting source compilation (this may take 20-60 minutes for both numpy and scipy)..."
        echo "  - This is normal behavior for ARM devices"
        echo "  - Compiling scipy from source (with numpy) requires significant resources"
        echo "  - Please be patient and do not interrupt the process"
        # Install with specific flags that are known to work on ARM platforms
        python3 -m pip install --user --no-cache-dir --no-binary numpy --force-reinstall numpy
        python3 -m pip install --user --no-cache-dir --no-binary scipy --force-reinstall scipy
      else
        echo "System dependencies check failed. Please install system dependencies first."
        exit 1
      fi
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