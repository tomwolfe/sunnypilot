#!/usr/bin/env bash

SP_C3_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
DIR="$( cd "$SP_C3_DIR/../../../.." >/dev/null 2>&1 && pwd )"

source "$SP_C3_DIR/launch_env.sh"

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0


  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$SP_C3_DIR/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find ${DIR}/.git -newer ${DIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $DIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $DIR
          cd $DIR

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # handle pythonpath
  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD"

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # Check and install Python dependencies if needed
  check_and_install_python_deps() {
    # Check for root privileges
    if [ "$EUID" -ne 0 ] && [ "$(id -u)" -ne 0 ]; then
      echo "Error: This script must be run as root"
      return 1
    fi

    # Check if pip is available for python3
    if ! python3 -m pip --version &> /dev/null; then
      echo "ERROR: pip is not installed for python3"
      return 1
    fi

    # Check if running in a virtual environment
    if [[ ! "$VIRTUAL_ENV" ]]; then
      # Check if virtual environment exists and activate it if it does
      VENV_PATH="$DIR/.venv"
      if [ -f "$VENV_PATH/bin/activate" ]; then
        echo "INFO: Virtual environment found at $VENV_PATH, activating..."
        source "$VENV_PATH/bin/activate"
        echo "INFO: Virtual environment activated"
      else
        echo "INFO: No virtual environment found at $VENV_PATH"
      fi
    fi

    # Save current directory and change to $DIR for operations
    local original_dir=$(pwd)
    cd $DIR

    # Create a Python script to check if all required packages from requirements.txt are available
    local python_check_script=$(cat << 'PYTHON_CHECK_EOF'
import sys
import re

def parse_requirements(file_path):
    """Parse requirements.txt and return a list of package names."""
    packages = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#') and not line.startswith(';'):
                # Extract package name (before ==, >=, <=, etc.)
                package = re.split(r'[>=<!=]+', line, 1)[0]
                package = package.strip()
                if package:
                    packages.append(package)
    return packages

def check_imports(packages):
    """Try to import each package and return list of missing packages."""
    missing_packages = []
    for package in packages:
        try:
            __import__(package)
        except ImportError:
            # Some packages have different import names than package names (e.g., 'scipy' vs 'scipy')
            # Try variations
            import_name = package.replace('-', '_')  # Common pattern
            try:
                __import__(import_name)
            except ImportError:
                missing_packages.append(package)
    return missing_packages

if __name__ == "__main__":
    requirements_file = sys.argv[1] if len(sys.argv) > 1 else "requirements.txt"

    try:
        packages = parse_requirements(requirements_file)
        missing = check_imports(packages)

        if missing:
            print(",".join(missing))
            sys.exit(1)  # Exit with error code if packages are missing
        else:
            print("all_found")
            sys.exit(0)  # Exit successfully if all packages are found
    except FileNotFoundError:
        print(f"ERROR: requirements.txt not found at {requirements_file}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
PYTHON_CHECK_EOF
)

    # Write the Python script to a temporary file and execute it
    local temp_script="/tmp/check_requirements_sp.py"
    echo "$python_check_script" > "$temp_script"

    local missing_packages=$(python3 "$temp_script" "$DIR/requirements.txt" 2>/dev/null)

    if [ "$missing_packages" != "all_found" ] && [ "$missing_packages" != "ERROR: requirements.txt not found at $DIR/requirements.txt" ]; then
      if [ -n "$missing_packages" ]; then
        echo "Missing Python packages detected: $missing_packages"
        echo "Installing Python dependencies from requirements.txt..."

        # Validate requirements.txt is readable and not empty
        if [ ! -r "requirements.txt" ]; then
          echo "Error: requirements.txt is not readable"
          cd "$original_dir"  # Return to original directory
          rm -f "$temp_script"  # Clean up the temporary script
          return 1
        fi

        if [ ! -s "requirements.txt" ]; then
          echo "Error: requirements.txt is empty"
          cd "$original_dir"  # Return to original directory
          rm -f "$temp_script"  # Clean up the temporary script
          return 1
        fi

        echo "Installing from requirements.txt..."

        # Ensure pip is up-to-date for compatibility
        python3 -m pip install --upgrade pip 2>/dev/null || echo "Warning: Could not upgrade pip, continuing with current version"

        # Check network connectivity before attempting online install
        if curl -s --max-time 5 https://pypi.org/simple/ &> /dev/null; then
          echo "Network available, proceeding with online installation..."
          if python3 -m pip install --no-cache-dir -r requirements.txt; then
            echo "Python dependencies installed."
          else
            echo "ERROR: Failed to install Python dependencies from $DIR/requirements.txt"
            cd "$original_dir"  # Return to original directory
            rm -f "$temp_script"  # Clean up the temporary script
            return 1
          fi
        else
          echo "Warning: No network connection available for installing dependencies."

          # Check if /data is writable before attempting to create cache directory
          if [ ! -w "/data" ]; then
            echo "Error: /data directory is not writable. Cannot create offline package cache."
            cd "$original_dir"  # Return to original directory
            rm -f "$temp_script"  # Clean up the temporary script
            return 1
          fi

          # Ensure the cache directory exists
          if [ ! -d "/data/python_packages" ]; then
            echo "Creating cache directory: /data/python_packages"
            mkdir -p /data/python_packages
          fi

          # Check if cache directory has files
          if [ -n "$(ls -A /data/python_packages 2>/dev/null)" ]; then
            echo "Attempting offline installation with cached packages..."
            # Try installing with --find-links if there are local wheels
            if python3 -m pip install --no-cache-dir --find-links /data/python_packages -r requirements.txt --no-index; then
              echo "Offline installation successful"
            else
              echo "Error: Could not install dependencies - no network and cached packages failed to install"
              cd "$original_dir"  # Return to original directory
              rm -f "$temp_script"  # Clean up the temporary script
              return 1
            fi
          else
            echo "Error: Could not install dependencies - no network and no cached packages available in /data/python_packages"
            echo "Please ensure offline packages are pre-loaded in /data/python_packages or connect to the internet."
            cd "$original_dir"  # Return to original directory
            rm -f "$temp_script"  # Clean up the temporary script
            return 1
          fi
        fi
      fi
    elif [ "$missing_packages" = "ERROR: requirements.txt not found at $DIR/requirements.txt" ]; then
      echo "ERROR: requirements.txt file not found at $DIR/requirements.txt"
      cd "$original_dir"  # Return to original directory
      rm -f "$temp_script"  # Clean up the temporary script
      return 1
    else
      echo "All Python dependencies are already installed."
    fi

    # Clean up the temporary script
    rm -f "$temp_script"

    # Return to original directory before exiting function
    cd "$original_dir"
    return 0
  }

  # Check and install Python dependencies before starting manager
  check_and_install_python_deps || exit 1

  # start manager
  cd $DIR/system/manager
  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  fi

  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
