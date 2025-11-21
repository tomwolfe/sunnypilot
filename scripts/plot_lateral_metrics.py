import pandas as pd
import matplotlib.pyplot as plt

def plot_lateral_metrics(file_path):
  """
  Parses the lateral control metrics log file and generates plots.

  Args:
    file_path: The path to the log file.
  """
  try:
    df = pd.read_csv(file_path)
  except FileNotFoundError:
    print(f"Error: Log file not found at {file_path}")
    return

  # Plot proportional gain vs. desired curvature
  plt.figure(1)
  plt.scatter(df["desired_curvature"], df["p_gain"])
  plt.xlabel("Desired Curvature")
  plt.ylabel("Proportional Gain")
  plt.title("Proportional Gain vs. Desired Curvature")
  plt.grid(True)

  # Plot output torque vs. desired curvature
  plt.figure(2)
  plt.scatter(df["desired_curvature"], df["output_torque"])
  plt.xlabel("Desired Curvature")
  plt.ylabel("Output Torque")
  plt.title("Output Torque vs. Desired Curvature")
  plt.grid(True)

  plt.show()

if __name__ == "__main__":
  # The log file is expected to be in /data/openpilot/metrics/lateral_control_metrics.csv on the device.
  # You can copy the file from the device to your computer to analyze it.
  # For example: scp user@hostname:/data/openpilot/metrics/lateral_control_metrics.csv .
  plot_lateral_metrics("lateral_control_metrics.csv")
