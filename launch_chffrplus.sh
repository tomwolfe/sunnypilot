#!/usr/bin/env bash

# Set DIR to the directory containing this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
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
  echo "Checking Python dependencies..."
  echo "INFO: Using directory $DIR for requirements.txt"
  if ! command -v python3 &> /dev/null; then
    echo "ERROR: python3 is not installed or not in PATH"
    exit 1
  fi

  # Check if pip is available for python3
  if ! python3 -m pip --version &> /dev/null; then
    echo "ERROR: pip is not installed for python3"
    exit 1
  fi

  # Check if running in a virtual environment
  if [[ "$VIRTUAL_ENV" ]]; then
    echo "INFO: Running in virtual environment: $VIRTUAL_ENV"
  else
    echo "INFO: Not running in a virtual environment, dependencies will be installed system-wide"
  fi

  # Create a Python script to check if all required packages from requirements.txt are available
  # Requirements file is expected to be at $DIR/requirements.txt where DIR is the script directory
  python_check_script=$(cat << 'PYTHON_CHECK_EOF'
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
                package = re.split(r'[>=<!=]', line, 1)[0]
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
  temp_script="/tmp/check_requirements.py"
  echo "$python_check_script" > "$temp_script"

  missing_packages=$(python3 "$temp_script" "$DIR/requirements.txt" 2>/dev/null)

  if [ "$missing_packages" != "all_found" ] && [ "$missing_packages" != "ERROR: requirements.txt not found at $DIR/requirements.txt" ]; then
    if [ -n "$missing_packages" ]; then
      echo "Missing Python packages detected: $missing_packages"
      echo "Installing Python dependencies from requirements.txt..."
      # Use python3 -m pip to ensure we're installing for the same interpreter being checked
      if python3 -m pip install -r "$DIR/requirements.txt"; then
        echo "Python dependencies installed."
      else
        echo "ERROR: Failed to install Python dependencies from $DIR/requirements.txt"
        exit 1 # Exit immediately on failure
      fi
    fi
  elif [ "$missing_packages" = "ERROR: requirements.txt not found at $DIR/requirements.txt" ]; then
    echo "ERROR: requirements.txt file not found at $DIR/requirements.txt"
    echo "INFO: DIR is set to: $DIR"
    exit 1
  else
    echo "All Python dependencies are already installed."
  fi

  # Clean up the temporary script
  rm -f "$temp_script"

  # start manager
  if ! cd system/manager; then
    echo "ERROR: system/manager directory not found"
    exit 1
  fi
  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
