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
1. Creating a `requirements.txt` file with all runtime dependencies, automatically generated from `pyproject.toml`
2. Modifying the installation script to install packages from `requirements.txt` as a fallback
3. Switching from Hatchling to Setuptools build backend for better OpenPilot compatibility

### Files Modified
- `pyproject.toml`: Changed build backend from Hatchling to Setuptools
- `requirements.txt`: Generated from pyproject.toml with all runtime dependencies
- `tools/install_python_dependencies.sh`: Added fallback installation from requirements.txt
- `tools/generate_requirements.py`: Script to automatically generate requirements.txt from pyproject.toml
- `tools/generate_requirements.sh`: Shell script to generate requirements.txt from pyproject.toml

### Dependency Management Process
**Important:** To maintain consistency, the `requirements.txt` file is now automatically generated from `pyproject.toml` to avoid dependency duplication and drift.

When adding or updating dependencies:
1. Update the dependencies in `pyproject.toml` under the `[project]` section
2. Regenerate `requirements.txt` using one of the following methods:
   - Using Python script: `python tools/generate_requirements.py`
   - Using shell script: `bash tools/generate_requirements.sh`

### Installation Process
The installation process now works as follows:
1. `uv sync` installs dependencies from `pyproject.toml`
2. `pip install -r requirements.txt` ensures all dependencies are available for direct execution

This ensures that dependencies are available both when installed as a package and when running directly from source on the device.

### Verification
To verify that `requirements.txt` is up-to-date with `pyproject.toml`, run:
`python tools/generate_requirements.py --verify`
This will return exit code 0 if they match, or 1 if they don't match.