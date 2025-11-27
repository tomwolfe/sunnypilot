#!/usr/bin/env python3
"""
Script to extract direct dependencies from pyproject.toml and write them to requirements.txt
This ensures consistency between the two dependency sources without including sub-dependencies.
"""
import toml
import sys
import os
from pathlib import Path


def extract_dependencies_from_pyproject():
    """Extract direct dependencies from pyproject.toml"""
    project_root = Path(__file__).parent.parent
    pyproject_path = project_root / "pyproject.toml"
    
    if not pyproject_path.exists():
        print(f"Error: {pyproject_path} not found")
        return []
    
    try:
        with open(pyproject_path, 'r') as f:
            pyproject_data = toml.load(f)
        
        # Extract dependencies from the [project] section
        dependencies = pyproject_data.get('project', {}).get('dependencies', [])
        
        # Extract optional dependencies (extras) if needed
        optional_deps = []
        for extra_name, extra_deps in pyproject_data.get('project', {}).get('optional-dependencies', {}).items():
            optional_deps.extend(extra_deps)
        
        # Combine all dependencies
        all_deps = list(set(dependencies + optional_deps))  # Use set to avoid duplicates
        
        return all_deps
    except Exception as e:
        print(f"Error reading pyproject.toml: {e}")
        return []


def write_requirements_txt(dependencies):
    """Write dependencies to requirements.txt"""
    project_root = Path(__file__).parent.parent
    requirements_path = project_root / "requirements.txt"
    
    try:
        with open(requirements_path, 'w') as f:
            for dep in sorted(dependencies, key=str.lower):
                f.write(f"{dep}\n")
        
        print(f"Successfully wrote {len(dependencies)} dependencies to {requirements_path}")
        return True
    except Exception as e:
        print(f"Error writing requirements.txt: {e}")
        return False


def main():
    dependencies = extract_dependencies_from_pyproject()
    
    if not dependencies:
        print("No dependencies found in pyproject.toml")
        return 1
    
    success = write_requirements_txt(dependencies)
    
    if success:
        print("requirements.txt has been updated from pyproject.toml")
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())