#!/usr/bin/env bash
# Script to generate requirements.txt from pyproject.toml

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/.."

cd "$PROJECT_ROOT"

echo "Generating requirements.txt from pyproject.toml..."

# Use the Python script to generate requirements.txt
python tools/generate_requirements.py