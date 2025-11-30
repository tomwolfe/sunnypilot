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

    # Install critical dependencies first to avoid conflicts
    echo "Installing critical dependencies (numpy, scipy) first..."
    echo "  - If this step seems to hang, it's likely compiling from source"
    echo "  - This is normal behavior for ARM devices and may take 10-30 minutes"

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

    # Verify critical packages were installed before continuing
    echo "Verifying critical package installation..."
    if python -c "import scipy; print('scipy version:', scipy.__version__)" && \
       python -c "import numpy; print('numpy version:', numpy.__version__)"; then
      echo "Critical packages (scipy, numpy) verified successfully."
    else
      echo "ERROR: Critical packages (scipy, numpy) could not be imported."
      echo "Installation may be incomplete. Please check the logs above."
      exit 1
    fi

    # Install all other dependencies from requirements.txt
    echo "Installing all other Python dependencies from requirements.txt..."
    echo "  - This may take several minutes depending on network speed and package complexity"
    echo "  - Some packages may require compilation on ARM devices"

    # Install remaining packages with a more permissive approach for the failing packages
    if pip install -r "$DIR/requirements.txt" --prefer-binary --timeout 300; then
      echo "Python dependencies installed successfully"
    else
      echo "WARNING: Some Python dependencies failed to install from requirements.txt."
      echo "  Attempting to continue with critical packages only..."

      # Ensure critical packages are still available after failed installation
      if ! python -c "import scipy; print('scipy version:', scipy.__version__)" 2>/dev/null; then
        echo "ERROR: Critical package scipy is not available after installation."
        exit 1
      fi
      if ! python -c "import numpy; print('numpy version:', numpy.__version__)" 2>/dev/null; then
        echo "ERROR: Critical package numpy is not available after installation."
        exit 1
      fi
      echo "Critical packages verified successfully."
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