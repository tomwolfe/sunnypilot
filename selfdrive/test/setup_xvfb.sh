#!/usr/bin/env bash

# Sets up a virtual display for running map renderer and simulator without an X11 display
# Also handles cleanup on script exit

DISP_ID=99
export DISPLAY=:$DISP_ID

# Cleanup function to kill Xvfb on exit
cleanup() {
  if [ -n "$XVFB_PID" ] && kill -0 "$XVFB_PID" 2>/dev/null; then
    echo "Stopping Xvfb (PID $XVFB_PID)..."
    kill -TERM "$XVFB_PID" 2>/dev/null
    sleep 0.5
    # Force kill if still running
    if kill -0 "$XVFB_PID" 2>/dev/null; then
      kill -KILL "$XVFB_PID" 2>/dev/null
    fi
    wait "$XVFB_PID" 2>/dev/null
  fi
  # Clean up X11 socket
  rm -f /tmp/.X11-unix/X$DISP_ID 2>/dev/null
  rm -f /tmp/.X11-lock 2>/dev/null
}

# Register cleanup on EXIT, INT, TERM
trap cleanup EXIT INT TERM

# Start Xvfb in background and capture PID
sudo Xvfb $DISPLAY -screen 0 2160x1080x24 2>/dev/null &
XVFB_PID=$!

# Wait for X11 socket to be available
timeout=30
elapsed=0
while [ ! -S /tmp/.X11-unix/X$DISP_ID ] && [ $elapsed -lt $timeout ]; do
  echo "Waiting for Xvfb..."
  sleep 1
  elapsed=$((elapsed + 1))
done

# Check if Xvfb started successfully
if [ ! -S /tmp/.X11-unix/X$DISP_ID ]; then
  echo "ERROR: Xvfb failed to start within ${timeout} seconds"
  cleanup
  exit 1
fi

touch ~/.Xauthority
export XDG_SESSION_TYPE="x11"
xset -q

echo "Xvfb started successfully on display $DISPLAY (PID: $XVFB_PID)"