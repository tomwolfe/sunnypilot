#!/usr/bin/env python3
# type: ignore

import os
import argparse
import numpy as np

import cereal.messaging as messaging
from openpilot.tools.lib.logreader import LogReader, ReadMode


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Analyze model execution times from a log file.")
  parser.add_argument("log_file", type=str, help="Path to the .rlog file to analyze.")
  args = parser.parse_args()

  log_path = args.log_file
  if not os.path.exists(log_path):
    print(f"Error: Log file not found at {log_path}")
    exit(1)

  execution_times = []

  # Read messages from the log file
  for msg in LogReader(log_path, ReadMode.RLOG):
    if msg.which() == 'modelV2':
      # Check if the message is valid and has modelExecutionTime
      if msg.modelV2.valid and msg.modelV2.modelExecutionTime > 0:
        execution_times.append(msg.modelV2.modelExecutionTime)

  if not execution_times:
    print(f"No modelV2 messages with valid execution times found in {log_path}")
    exit(0)

  # Convert to numpy array and milliseconds
  execution_times_ms = np.array(execution_times) * 1000

  print(f"\n\nAnalysis of model execution times from {log_path}:")
  print(f"\tNumber of samples: {len(execution_times_ms)}")
  print(f"\tAverage: {np.mean(execution_times_ms):0.2f}ms")
  print(f"\tMin: {np.min(execution_times_ms):0.2f}ms")
  print(f"\tMax: {np.max(execution_times_ms):0.2f}ms")
  print(f"\tMedian: {np.median(execution_times_ms):0.2f}ms")
  print(f"\t95th percentile: {np.percentile(execution_times_ms, 95):0.2f}ms")
  print(f"\t50ms Threshold Violations: {np.sum(execution_times_ms > 50)} samples ({np.mean(execution_times_ms > 50) * 100:0.2f}%)")
  print("\n\n")
