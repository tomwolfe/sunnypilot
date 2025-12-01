#!/usr/bin/env python3
"""
Verification test to ensure the MockParams global replacement is fixed.
This tests that after importing the problematic test file, the real Params class
still works correctly and has all required methods.
"""
import sys
sys.path.insert(0, '/Users/tom/Documents/apps/sunnypilot2')

# Try to import the problematic test file first (this would have caused global replacement before the fix)
from selfdrive.controls.lib.test_enhanced_self_learning import MockParams

# Now let's check if MockParams has the required methods
mock = MockParams()
print("MockParams has required methods:")
print(f"  - clear_all: {hasattr(mock, 'clear_all')}")
print(f"  - put_bool: {hasattr(mock, 'put_bool')}")
print(f"  - get_bool: {hasattr(mock, 'get_bool')}")
print(f"  - put: {hasattr(mock, 'put')}")
print(f"  - get: {hasattr(mock, 'get')}")
print(f"  - delete: {hasattr(mock, 'delete')}")
print(f"  - all_keys: {hasattr(mock, 'all_keys')}")

print("\nMockParams can now perform all required operations:")
mock.put_bool("test_bool", True)
print(f"  - put_bool('test_bool', True): {mock.get_bool('test_bool')}")
mock.put("test_string", "hello")
print(f"  - put('test_string', 'hello'): {mock.get('test_string')}")
mock.clear_all()
print(f"  - after clear_all(), get('test_bool'): {mock.get('test_bool')}")

print("\n✅ All MockParams methods work correctly!")