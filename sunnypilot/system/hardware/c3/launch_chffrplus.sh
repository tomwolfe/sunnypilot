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

    # Save current directory and change to $DIR for operations
    local original_dir=$(pwd)
    cd $DIR

    # Check if key Python dependencies are available (comprehensive check beyond just scipy)
    local required_modules=("scipy" "numpy" "requests")
    local missing_modules=()

    for module in "${required_modules[@]}"; do
      if ! python3 -c "import $module" 2>/dev/null; then
        missing_modules+=("$module")
      fi
    done

    # If no key modules are missing, continue
    if [ ${#missing_modules[@]} -eq 0 ]; then
      echo "All key Python dependencies are available, continuing..."
      cd "$original_dir"  # Return to original directory
      return 0
    else
      echo "Missing Python dependencies: ${missing_modules[*]}, installing..."
      # Fallback: install directly from requirements.txt (removed insecure external script call)
      if [ -f "requirements.txt" ]; then
        echo "Installing from requirements.txt..."

        # Ensure pip is up-to-date for compatibility
        python3 -m pip install --upgrade pip 2>/dev/null || echo "Warning: Could not upgrade pip, continuing with current version"

        # Check network connectivity before attempting online install
        if ping -c 1 8.8.8.8 &> /dev/null; then
          echo "Network available, proceeding with online installation..."
          python3 -m pip install --no-cache-dir -r requirements.txt
        else
          echo "Warning: No network connection available for installing dependencies."

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
              return 1
            fi
          else
            echo "Error: Could not install dependencies - no network and no cached packages available in /data/python_packages"
            echo "Please ensure offline packages are pre-loaded in /data/python_packages or connect to the internet."
            cd "$original_dir"  # Return to original directory
            return 1
          fi
        fi
      else
        echo "Error: requirements.txt not found, cannot install dependencies"
        cd "$original_dir"  # Return to original directory
        return 1
      fi
    fi

    # Return to original directory before exiting function
    cd "$original_dir"
    return 0
  }

  # start manager
  cd $DIR/system/manager
  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  fi

  # Check and install Python dependencies before starting manager
  check_and_install_python_deps || exit 1

  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
