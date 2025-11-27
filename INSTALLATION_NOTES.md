# Installation Notes

## Fix for "ModuleNotFoundError: scipy" Issue

### Problem
When running Sunnypilot on the device, users may encounter a "ModuleNotFoundError: scipy" error (along with other missing dependencies like numpy, etc.).

### Root Cause
The issue occurs because:
1. The project uses a PEP-517 build backend (setuptools)
2. The device runs the code directly from the source without installing it as a wheel package
3. Runtime dependencies declared in `pyproject.toml` are not automatically installed when running from source

### Solution
This has been fixed by:
1. Creating a `requirements.txt` file with all runtime dependencies
2. Modifying the installation script to install packages from `requirements.txt` as a fallback
3. Switching from Hatchling to Setuptools build backend for better OpenPilot compatibility

### Files Modified
- `pyproject.toml`: Changed build backend from Hatchling to Setuptools
- `requirements.txt`: Added all runtime dependencies
- `tools/install_python_dependencies.sh`: Added fallback installation from requirements.txt

### Installation Process
The installation process now works as follows:
1. `uv sync` installs dependencies from `pyproject.toml`
2. `pip install -r requirements.txt` ensures all dependencies are available for direct execution

This ensures that dependencies are available both when installed as a package and when running directly from source on the device.