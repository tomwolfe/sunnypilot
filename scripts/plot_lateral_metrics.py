import pandas as pd
import matplotlib.pyplot as plt
import argparse
import json
import numpy as np

def plot_lateral_metrics(file_path, config_json=None):
  """
  Parses the lateral control metrics log file and generates plots.

  Args:
    file_path: The path to the log file.
    config_json: An optional JSON string representing the configured curvature gain interpolation curve.
                 Format: "[[curvatures], [gains]]"
  """
  try:
    df = pd.read_csv(file_path)
  except FileNotFoundError:
    print(f"Error: Log file not found at {file_path}")
    return

  # Plot proportional gain vs. desired curvature
  plt.figure(1)
  plt.scatter(df["desired_curvature"], df["p_gain"], label="Actual P-Gain")
  plt.xlabel("Desired Curvature")
  plt.ylabel("Proportional Gain")
  plt.title("Proportional Gain vs. Desired Curvature")
  plt.grid(True)

  if config_json:
    try:
      config_data = json.loads(config_json)
      if (isinstance(config_data, list) and len(config_data) == 2 and
          all(isinstance(l, list) and all(isinstance(f, (float, int)) for f in l) for l in config_data) and
          len(config_data[0]) > 0 and len(config_data[0]) == len(config_data[1])):
        
        # Sort data for proper interpolation if it's not already
        curvatures = np.array(config_data[0])
        gains = np.array(config_data[1])
        sort_indices = np.argsort(curvatures)
        curvatures = curvatures[sort_indices]
        gains = gains[sort_indices]

        # Generate points for the configured curve
        # Use the range of actual desired_curvature for plotting
        min_curv = df["desired_curvature"].min() if not df["desired_curvature"].empty else 0
        max_curv = df["desired_curvature"].max() if not df["desired_curvature"].empty else 0.1
        plot_curvatures = np.linspace(min_curv, max_curv, 100)
        
        # Ensure plot_curvatures doesn't exceed the configured_curvatures range for interp
        # If plot_curvatures go beyond, np.interp will extrapolate, which might not be desired.
        # Clamp to the range of configured curvatures.
        plot_curvatures = np.clip(plot_curvatures, curvatures.min(), curvatures.max())
        
        configured_gains = np.interp(plot_curvatures, curvatures, gains)
        plt.plot(plot_curvatures, configured_gains, color='red', linestyle='--', label='Configured Gain Curve')
        plt.legend()
      else:
        print(f"Warning: Invalid format for --config-json: {config_json}. Expected '[[curvatures], [gains]]'. Skipping plot.")
    except json.JSONDecodeError:
      print(f"Warning: Could not decode --config-json string. Skipping plot.")

  # Plot output torque vs. desired curvature
  plt.figure(2)
  plt.scatter(df["desired_curvature"], df["output_torque"])
  plt.xlabel("Desired Curvature")
  plt.ylabel("Output Torque")
  plt.title("Output Torque vs. Desired Curvature")
  plt.grid(True)

  plt.show()

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Plot lateral control metrics from a log file.")
  parser.add_argument("--file", type=str, default="lateral_control_metrics.csv",
                      help="Path to the lateral control metrics CSV file.")
  parser.add_argument("--config-json", type=str,
                      help="JSON string for curvature gain configuration, e.g., '[[0.0, 0.02], [1.0, 1.2]]'")
  args = parser.parse_args()

  plot_lateral_metrics(args.file, args.config_json)
