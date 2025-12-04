#!/usr/bin/env bash
set -e

# Increase the pip timeout to handle TimeoutError
export PIP_DEFAULT_TIMEOUT=200

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
ROOT="$DIR"/../
cd "$ROOT"

if ! command -v "uv" > /dev/null 2>&1; then
  echo "installing uv..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
  UV_BIN="$HOME/.local/bin"
  PATH="$UV_BIN:$PATH"
fi

echo "updating uv..."
# ok to fail, can also fail due to installing with brew
uv self update || true

echo "installing python packages..."
uv sync --frozen --all-extras
source .venv/bin/activate

# As a fallback, also install from requirements.txt to ensure compatibility with direct execution
if [ -f "$ROOT/requirements.txt" ]; then
  echo "installing additional packages from requirements.txt..."
  if [[ "$(uname)" == 'Darwin' ]]; then
    # On macOS, filter out av package due to ffmpeg compatibility issues
    # Create a temporary file with the av package line commented out
    sed 's/^av;/# av;/' "$ROOT/requirements.txt" > /tmp/requirements_filtered.txt
    pip install -r /tmp/requirements_filtered.txt
    rm -f /tmp/requirements_filtered.txt
  else
    pip install -r "$ROOT/requirements.txt"
  fi
fi

if [[ "$(uname)" == 'Darwin' ]]; then
  touch "$ROOT"/.env
  echo "# msgq doesn't work on mac" >> "$ROOT"/.env
  echo "export ZMQ=1" >> "$ROOT"/.env
  echo "export OBJC_DISABLE_INITIALIZE_FORK_SAFETY=YES" >> "$ROOT"/.env
fi
