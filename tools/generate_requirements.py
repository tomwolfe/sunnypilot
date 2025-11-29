#!/usr/bin/env python3
"""
Script to generate requirements.txt from pyproject.toml
This ensures consistency between the two dependency sources.
"""
import subprocess
import sys
import os
import toml
from pathlib import Path


def generate_requirements_txt():
    """Generate requirements.txt from pyproject.toml by extracting all dependencies"""
    try:
        # Change to project root directory
        project_root = Path(__file__).parent.parent
        os.chdir(project_root)

        # Extract direct dependencies from pyproject.toml
        pyproject_path = project_root / "pyproject.toml"
        with open(pyproject_path) as f:
            pyproject_data = toml.load(f)

        # Extract dependencies from the [project] section
        all_deps = set()

        # Add main dependencies
        dependencies = pyproject_data.get('project', {}).get('dependencies', [])
        all_deps.update(dependencies)

        # Add optional dependencies (testing, dev, tools, etc.)
        optional_deps = pyproject_data.get('project', {}).get('optional-dependencies', {})
        for extra_deps in optional_deps.values():
            all_deps.update(extra_deps)

        # Write dependencies to requirements.txt
        requirements_path = project_root / "requirements.txt"
        with open(requirements_path, 'w') as f:
            for dep in sorted(all_deps, key=str.lower):
                f.write(f"{dep}\n")

        print("Successfully generated requirements.txt from pyproject.toml")
        return True

    except Exception as e:
        print(f"Error generating requirements.txt: {e}")
        return False


def verify_dependencies_match():
    """Verify that pyproject.toml and requirements.txt have consistent dependencies"""
    try:
        project_root = Path(__file__).parent.parent
        pyproject_path = project_root / "pyproject.toml"

        # Extract dependencies from pyproject.toml
        with open(pyproject_path) as f:
            pyproject_data = toml.load(f)

        # Get all dependencies (main + optional)
        all_pyproject_deps = set()

        # Add main dependencies
        dependencies = pyproject_data.get('project', {}).get('dependencies', [])
        all_pyproject_deps.update(dependencies)

        # Add optional dependencies
        optional_deps = pyproject_data.get('project', {}).get('optional-dependencies', {})
        for extra_deps in optional_deps.values():
            all_pyproject_deps.update(extra_deps)

        # Read dependencies from requirements.txt
        requirements_path = project_root / "requirements.txt"
        with open(requirements_path) as f:
            req_deps = {line.strip() for line in f if line.strip() and not line.startswith('#')}

        return all_pyproject_deps == req_deps

    except Exception as e:
        print(f"Error verifying dependencies: {e}")
        return False


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--verify":
        # Just verify if the files match
        if verify_dependencies_match():
            print("✓ Dependencies in pyproject.toml match requirements.txt")
            sys.exit(0)
        else:
            print("✗ Dependencies in pyproject.toml do NOT match requirements.txt")
            sys.exit(1)
    else:
        # Generate requirements.txt from pyproject.toml
        success = generate_requirements_txt()
        if success:
            print("requirements.txt has been updated from pyproject.toml")
        else:
            sys.exit(1)