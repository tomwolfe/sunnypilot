class WMACConstants:
  # Lead detection parameters
  LEAD_WINDOW_SIZE = 6  # Stable detection window
  LEAD_PROB = 0.45  # Balanced threshold for lead detection

  # Slow down detection parameters - improved for traffic light/stop sign scenarios
  SLOW_DOWN_WINDOW_SIZE = 3  # More responsive for traffic lights/stops
  SLOW_DOWN_PROB = 0.25  # Lower threshold for earlier detection of slow-downs

  # Optimized slow down distance curve - enhanced for traffic light/stop sign scenarios
  # Adjusted to better anticipate stops at traffic lights and stop signs
  SLOW_DOWN_BP = [0., 10., 20., 30., 40., 50., 55., 60.]
  SLOW_DOWN_DIST = [25., 35., 50., 70., 90., 115., 135., 155.]  # Reduced distances for more responsive stopping

  # Slowness detection parameters
  SLOWNESS_WINDOW_SIZE = 8  # Slightly more responsive
  SLOWNESS_PROB = 0.50  # Lower threshold for better detection of slow traffic
  SLOWNESS_CRUISE_OFFSET = 1.025  # Conservative cruise speed offset
