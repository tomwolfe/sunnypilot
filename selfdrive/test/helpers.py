import contextlib
import http.server
import os
import signal
import threading
import time
import pytest
import psutil

from functools import wraps

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.system.manager.process_config import managed_processes
from openpilot.system.version import training_version, terms_version


def set_params_enabled():
  os.environ['FINGERPRINT'] = "TOYOTA_COROLLA_TSS2"
  os.environ['LOGPRINT'] = "debug"

  params = Params()
  params.put("HasAcceptedTerms", terms_version)
  params.put("CompletedTrainingVersion", training_version)
  params.put_bool("OpenpilotEnabledToggle", True)

  # valid calib
  msg = messaging.new_message('liveCalibration')
  msg.liveCalibration.validBlocks = 20
  msg.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
  params.put("CalibrationParams", msg.to_bytes())

def release_only(f):
  @wraps(f)
  def wrap(self, *args, **kwargs):
    if "RELEASE" not in os.environ:
      pytest.skip("This test is only for release branches")
    f(self, *args, **kwargs)
  return wrap


def _kill_process_hard(proc, name, timeout=5):
  """Forcefully kill a process, trying SIGTERM first then SIGKILL."""
  if proc is None or proc.proc is None:
    return
  
  try:
    # Try graceful shutdown first with SIGTERM
    proc.stop(sig=signal.SIGTERM, block=False)
    
    # Wait for process to exit
    start_time = time.monotonic()
    while time.monotonic() - start_time < timeout and proc.proc.exitcode is None:
      time.sleep(0.1)
    
    # If still running, send SIGKILL
    if proc.proc.exitcode is None:
      proc.stop(sig=signal.SIGKILL, block=True)
  except Exception:
    pass
  
  # Extra safety: use psutil to kill if still alive
  try:
    if proc.proc and proc.proc.pid is not None:
      psutil_proc = psutil.Process(proc.proc.pid)
      if psutil_proc.is_running():
        psutil_proc.kill()
        psutil_proc.wait(timeout=2)
  except (psutil.NoSuchProcess, psutil.TimeoutExpired, Exception):
    pass


@contextlib.contextmanager
def processes_context(processes, init_time=0, ignore_stopped=None):
  ignore_stopped = [] if ignore_stopped is None else ignore_stopped
  started_procs = []

  try:
    # start and assert started
    for n, p in enumerate(processes):
      managed_processes[p].start()
      started_procs.append(managed_processes[p])
      if n < len(processes) - 1:
        time.sleep(init_time)

    assert all(managed_processes[name].proc.exitcode is None for name in processes)

    yield [managed_processes[name] for name in processes]
    # assert processes are still started
    assert all(managed_processes[name].proc.exitcode is None for name in processes if name not in ignore_stopped)
  finally:
    # Ensure all processes are properly killed
    for p in processes:
      _kill_process_hard(managed_processes[p], p)


def with_processes(processes, init_time=0, ignore_stopped=None):
  def wrapper(func):
    @wraps(func)
    def wrap(*args, **kwargs):
      with processes_context(processes, init_time, ignore_stopped):
        return func(*args, **kwargs)

    return wrap
  return wrapper


def noop(*args, **kwargs):
  pass


def read_segment_list(segment_list_path):
  with open(segment_list_path) as f:
    seg_list = f.read().splitlines()

  return [(platform[2:], segment) for platform, segment in zip(seg_list[::2], seg_list[1::2], strict=True)]


@contextlib.contextmanager
def http_server_context(handler, setup=None):
  host = '127.0.0.1'
  server = http.server.HTTPServer((host, 0), handler)
  port = server.server_port
  t = threading.Thread(target=server.serve_forever)
  t.start()

  if setup is not None:
    setup(host, port)

  try:
    yield (host, port)
  finally:
    server.shutdown()
    server.server_close()
    t.join()


def with_http_server(func, handler=http.server.BaseHTTPRequestHandler, setup=None):
  @wraps(func)
  def inner(*args, **kwargs):
    with http_server_context(handler, setup) as (host, port):
      return func(*args, f"http://{host}:{port}", **kwargs)
  return inner


def DirectoryHttpServer(directory) -> type[http.server.SimpleHTTPRequestHandler]:
  # creates an http server that serves files from directory
  class Handler(http.server.SimpleHTTPRequestHandler):
    API_NO_RESPONSE = False
    API_BAD_RESPONSE = False

    def do_GET(self):
      if self.API_NO_RESPONSE:
        return

      if self.API_BAD_RESPONSE:
        self.send_response(500, "")
        return
      super().do_GET()

    def __init__(self, *args, **kwargs):
      super().__init__(*args, directory=str(directory), **kwargs)

  return Handler
