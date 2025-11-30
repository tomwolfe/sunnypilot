#!/usr/bin/env bash

set -e

echo "Fixing scipy installation on comma device..."

# Check if running on device
if [ ! -f /TICI ] && [ ! -f /EON ]; then
  echo "This script should be run on the comma device"
  exit 1
fi

echo "Installing system dependencies for scipy..."

# Update package list and install build dependencies
sudo apt update
sudo apt install -y python3-dev python3-numpy python3-setuptools python3-wheel python3-pip build-essential gfortran libopenblas-dev liblapack-dev libatlas-base-dev g++ pkg-config

# Upgrade pip to latest version
python3 -m pip install --upgrade pip

# Install or reinstall scipy with specific flags for ARM architecture
echo "Attempting to install scipy..."

# Try installing scipy directly first
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

# Verify installation
echo "Verifying scipy installation..."
if python3 -c "import scipy; print(f'scipy version: {scipy.__version__}')"; then
  echo "scipy installation verified successfully!"
else
  echo "scipy installation failed or not accessible"
  exit 1
fi

echo "scipy installation fix completed!"