import os

# Check if CI environment variable is set
if os.environ.get("CI"):
  BASEDIR = "/tmp/openpilot"
else:
  BASEDIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../"))
