import threading
import time
from openpilot.common.swaglog import cloudlog

def start_background_task(func, *args, name=None, daemon=True, delay_sec=0):
  """
  Starts a function in a new daemon thread after an optional delay.

  Args:
    func: The function to execute in the background.
    *args: Arguments to pass to the function.
    name: Optional name for the thread.
    daemon: If True, the thread will terminate when the main program finishes.
    delay_sec: Optional delay in seconds before starting the function.
  """
  def wrapper():
    if delay_sec > 0:
      time.sleep(delay_sec)
    try:
      func(*args)
    except Exception:
      cloudlog.exception(f"Error in background task {name or func.__name__}")

  thread = threading.Thread(target=wrapper, name=name or func.__name__, daemon=daemon)
  thread.start()
  return thread
