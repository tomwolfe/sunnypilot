# Sunnypilot Development Guide

This guide provides information about the development environment setup and CI/CD pipeline changes in sunnypilot.

## CI/CD Pipeline Changes

### Git Safe Directory Configuration

GitHub Actions runners on `ubuntu-24.04` and similar environments now enforce Git's "safe directory" security policy. This blocks `git` commands from running in directories owned by a different user (like `/tmp/openpilot/panda`, which is likely owned by root during the CI setup).

#### The Fix
Added `git config --global --add safe.directory /tmp/openpilot/panda` to every `run` command in `.github/workflows/tests.yaml` that executes Git-related tasks.

#### Impact
This ensures CI/CD pipeline stability by allowing Git commands to run in the temporary CI environment.

### Additional CI Improvements
- Added D-Bus service startup for system communication
- Added asset copying to ensure UI assets are available during tests
- X11/Xlib warnings are now filtered in CI logs to reduce noise

## BASEDIR Environment Handling

### The Change
The `common/basedir.py` file now checks for the `CI` environment variable:

```python
# Check if CI environment variable is set
if os.environ.get("CI"):
  BASEDIR = "/tmp/openpilot"
else:
  BASEDIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../"))
```

### Rationale
Local development and CI environments have different directory structures. The CI environment checks out code to `/tmp/openpilot`, while local development uses the repository's root. This change ensures all file path lookups work correctly in both contexts.

## System Robustness Improvements

### Wifi Manager Protection
Added validation to ensure that D-Bus object paths start with '/' to prevent crashes if NetworkManager returns malformed data.

### Application Image Handling
Added checks for zero-dimension images to prevent crashes during resizing operations.

### X11/Xlib Warning Filtering
Added filtering for harmless X11/Xlib warnings in CI logs to reduce noise.

## Development Best Practices

### Parameter Definitions
When adding new parameters to `params_keys.h`, ensure:
- The parameter has appropriate persistence flags (PERSISTENT, BACKUP, etc.)
- The parameter has a sensible default value
- The parameter is documented in this guide
- The parameter is properly validated in UI components

### UI Component Usage
When creating settings UI:
- Use `toggle_item_sp` for boolean parameters
- Use `option_item_sp` for numeric parameters with ranges
- Use `button_item` for actions that require callbacks
- Use `InputDialogSP` for text input operations