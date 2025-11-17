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

# Install tinygrad submodule
if [ -d "tinygrad_repo" ]; then
  echo "Installing tinygrad..."
  cd tinygrad_repo
  # Use normal installation in CI to avoid segfaults during model compilation
  if [ "$CI" = "true" ] || [ ! -z "$GITHUB_ACTIONS" ]; then
    echo "CI environment detected - installing tinygrad normally to avoid segfaults..."
    pip install .
  else
    echo "Installing tinygrad in development mode..."
    pip install -e .
  fi
  cd ..
fi

if [[ "$(uname)" == 'Darwin' ]]; then
  touch "$ROOT"/.env
  echo "# msgq doesn't work on mac" >> "$ROOT"/.env
  echo "export ZMQ=1" >> "$ROOT"/.env
  echo "export OBJC_DISABLE_INITIALIZE_FORK_SAFETY=YES" >> "$ROOT"/.env
fi
