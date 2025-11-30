#!/usr/bin/env bash

set -e

# Set DIR to the directory containing this script
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

echo "Starting sunnypilot installation..."

# Check if running on device
if [ -f /TICI ] || [ -f /EON ]; then
  echo "Running on comma device"

  # Install system dependencies for scipy and other C/C++ extensions
  echo "Installing system build dependencies for scipy and other extensions..."
  sudo apt update
  sudo apt install -y python3-dev python3-numpy python3-setuptools python3-wheel build-essential gfortran libopenblas-dev liblapack-dev libatlas-base-dev g++ libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev

  # Create a dedicated, large temporary directory on /data
  TEMP_DIR="/data/tmp"
  mkdir -p "$TEMP_DIR"
  echo "Using $TEMP_DIR as the temporary directory for pip downloads and builds."

  # Create a virtual environment in the project directory
  VENV_DIR="$DIR/.venv"
  echo "Creating virtual environment at $VENV_DIR..."
  python3 -m venv "$VENV_DIR"

  # Activate the virtual environment
  source "$VENV_DIR/bin/activate"

  # Set the temporary directory for this session to the large /data location
  export TMPDIR="$TEMP_DIR"
  echo "Set TMPDIR to $TMPDIR"

  # Install Python dependencies
  echo "Installing Python dependencies..."
  if [ -f "$DIR/requirements.txt" ]; then
    # Ensure pip is available in the virtual environment
    if ! command -v pip &> /dev/null; then
      echo "pip not found in virtual environment, attempting to bootstrap..."
      if python3 -m ensurepip --upgrade; then
        echo "ensurepip completed successfully"
      else
        echo "ensurepip failed, attempting manual pip installation..."
        # Fallback: download get-pip.py and install
        if curl -s https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py; then
          python3 /tmp/get-pip.py
          rm /tmp/get-pip.py
          echo "pip installed via get-pip.py"
        else
          echo "ERROR: Could not install pip in the virtual environment"
          echo "Manual intervention required. Try running: python3 -m ensurepip --upgrade"
          exit 1
        fi
      fi
    fi

    # Install scipy first to avoid conflicts
    echo "Installing scipy first (this may take several minutes for compilation)..."
    echo "  - If this step seems to hang, it's likely compiling scipy from source"
    echo "  - This is normal behavior for ARM devices and may take 10-30 minutes"
    if pip install scipy; then
      echo "scipy installed successfully"
    else
      echo "Failed to install scipy individually, attempting source compilation..."
      echo "  - This will compile scipy from source without binary packages"
      echo "  - Expected duration: 20-45 minutes on ARM devices"
      echo "  - This is the most reliable method for ARM architecture"
      if pip install --no-binary scipy --force-reinstall scipy; then
        echo "scipy source compilation completed successfully"
      else
        echo "ERROR: scipy installation failed completely."
        echo "Please run fix_scipy_install.sh to attempt recovery."
        exit 1
      fi
    fi

    # Install all other dependencies from requirements.txt
    echo "Installing all other Python dependencies from requirements.txt..."
    echo "  - This may take several minutes depending on network speed and package complexity"
    echo "  - Some packages may require compilation on ARM devices"
    if pip install -r "$DIR/requirements.txt"; then
      echo "Python dependencies installed successfully"
    else
      echo "ERROR: Failed to install Python dependencies from requirements.txt."
      echo "This is likely due to a network issue or a package incompatibility."
      echo "Please check your internet connection and try again."
      exit 1
    fi

    # Verify critical packages were installed
    echo "Verifying critical package installation..."
    if python -c "import scipy; print('scipy version:', scipy.__version__)" && \
       python -c "import numpy; print('numpy version:', numpy.__version__)"; then
      echo "Critical packages (scipy, numpy) verified successfully."
    else
      echo "ERROR: Critical packages (scipy, numpy) could not be imported."
      echo "Installation may be incomplete. Please check the logs above."
      exit 1
    fi

  else
    echo "No requirements.txt found, skipping dependency installation"
  fi

  # Install with uv as fallback (if available)
  if command -v uv &> /dev/null; then
    echo "Installing with uv..."
    # Use the active virtual environment with locked dependencies for reproducibility
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
  echo ""
  echo "IMPORTANT: To use sunnypilot, always activate the virtual environment first:"
  echo "  source $VENV_DIR/bin/activate"
  echo "Then run your sunnypilot commands."
else
  echo "Not running on device, skipping device-specific installation"
fi