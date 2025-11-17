#!/bin/bash
# Script to apply patches to submodules after they are checked out

echo "Applying patches to submodules..."

# Apply tinygrad math library patch
if [ -f "patches/tinygrad-math-lib-fix.patch" ]; then
    echo "Applying tinygrad math library patch..."
    cd tinygrad_repo
    git apply ../patches/tinygrad-math-lib-fix.patch
    if [ $? -eq 0 ]; then
        echo "Patch applied successfully to tinygrad_repo"
        # Commit the change so the submodule points to the patched version locally
        git add tinygrad/runtime/support/elf.py
        git commit -m "Apply math library fix for fmaxf symbol"
    else
        echo "Failed to apply patch to tinygrad_repo"
        exit 1
    fi
    cd ..
else
    echo "Patch file not found: patches/tinygrad-math-lib-fix.patch"
fi

echo "All patches applied."