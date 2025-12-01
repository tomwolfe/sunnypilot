class WMACConstants:
  # Lead detection parameters
  LEAD_WINDOW_SIZE = 6  # Stable detection window
  LEAD_PROB = 0.45  # Balanced threshold for lead detection

  # Slow down detection parameters - improved for traffic light/stop sign scenarios
  SLOW_DOWN_WINDOW_SIZE = 3  # More responsive for traffic lights/stops
  SLOW_DOWN_PROB = 0.25  # Lower threshold for earlier detection of slow-downs

  # Threshold multipliers for different scenarios
  SLOW_DOWN_THRESHOLD_MULTIPLIER = 0.7  # Multiplier for slow down detection threshold with map data integration

  # Optimized slow down distance curve - enhanced for traffic light/stop sign scenarios
  # Adjusted to better anticipate stops at traffic lights and stop signs
  SLOW_DOWN_BP = [0., 10., 20., 30., 40., 50., 55., 60.]
  SLOW_DOWN_DIST = [25., 35., 50., 70., 90., 115., 135., 155.]  # Reduced distances for more responsive stopping

  # Traffic light/stop sign related constants
  TRAFFIC_LIGHT_URGENCY_MULTIPLIER = 1.2  # Multiplier for urgency calculation when approaching traffic light
  STOP_SIGN_URGENCY_MULTIPLIER = 1.3  # Multiplier for urgency calculation when approaching stop sign
  TRAFFIC_LIGHT_CLOSE_DISTANCE_RATIO = 0.3  # Ratio to consider traffic light as 'very close'
  INTERSECTION_APPROACH_MID_POINT_RATIO = 0.5  # Ratio to consider approaching mid-point of expected distance
  URBAN_INTERSECTION_MIN_SPEED = 20.0  # Minimum speed for urban intersection logic (in kph)
  URBAN_INTERSECTION_MAX_SPEED = 35.0  # Maximum speed for urban intersection logic (in kph)
  URBAN_INTERSECTION_SPEED_FACTOR_DENOMINATOR = 50.0  # Denominator for urban intersection speed factor calculation

  # Slowness detection parameters
  SLOWNESS_WINDOW_SIZE = 8  # Slightly more responsive
  SLOWNESS_PROB = 0.50  # Lower threshold for better detection of slow traffic
  SLOWNESS_CRUISE_OFFSET = 1.025  # Conservative cruise speed offset
