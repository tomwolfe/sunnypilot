# Submodule Patches

This directory contains patches for submodules that need fixes not available in the upstream repositories.

## Current Patches

### tinygrad-math-lib-fix.patch
- **Issue**: CI build failures due to undefined `fmaxf` symbol during tinygrad model compilation
- **Fix**: Adds math library (`libm`) to the JIT loader by default in `elf.py`
- **File**: `tinygrad/runtime/support/elf.py`

## Applying Patches

Patches are applied automatically using the `scripts/apply_submodule_patches.sh` script.
This script should be run after submodules are checked out during the build process.