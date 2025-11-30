import threading
import time
from datetime import datetime

from cereal import messaging
from openpilot.common.params import Params

# For calculating distance from lat/lon
from geopy.distance import geodesic

class TripDataCollector:
  def __init__(self):
    self._is_trip_active = False
    self._trip_start_time = None
    self._trip_end_time = None
    self._last_car_state = None
    self._last_location = None
    self._total_distance_meters = 0.0
    self._speed_samples = [] # List of (timestamp, speed_kph)
    self._route_points = []  # List of (latitude, longitude)
    self._vin = "N/A"
    self._car_model = "N/A"
    self._start_fuel_level = None # Percentage 0.0-1.0
    self._end_fuel_level = None # Percentage 0.0-1.0

    self._params = Params()
    self._lock = threading.Lock() # Lock for accessing trip data

    # Create and start the background thread
    self._thread = threading.Thread(target=self._run_collector, daemon=True)
    self._thread.start()

  def _reset_trip_data(self):
    with self._lock:
      self._is_trip_active = False
      self._trip_start_time = None
      self._trip_end_time = None
      self._last_car_state = None
      self._last_location = None
      self._total_distance_meters = 0.0
      self._speed_samples = []
      self._route_points = []
      self._start_fuel_level = None
      self._end_fuel_level = None

  def _run_collector(self):
    car_state_sock = messaging.sub_sock('carState', timeout=50)
    live_location_kalman_sock = messaging.sub_sock('liveLocationKalman', timeout=50)

    while True:
      # Process carState messages
      msgs = messaging.drain_sock(car_state_sock)
      for msg in msgs:
        cs = msg.carState
        self._process_car_state(cs)

      # Process liveLocationKalman messages
      msgs = messaging.drain_sock(live_location_kalman_sock)
      for msg in msgs:
        llk = msg.liveLocationKalman
        self._process_live_location_kalman(llk)

      time.sleep(0.01) # Small delay to prevent busy-waiting

  def _process_car_state(self, cs):
    current_time = datetime.now() # Use system time for now, can switch to monotonic time later if needed

    # Detect trip start/end based on standstill
    # A trip starts when not standstill and was previously standstill (or not active)
    # A trip ends when standstill and was previously not standstill (and active)

    vin = self._params.get("CarParams", encoding='utf8') # CarParams is a bytes object, not a string
    car_model = self._params.get("CarParams", encoding='utf8')
    if vin:
        try:
            car_params = messaging.log_from_bytes(vin, messaging.log.Car.CarParams) # Assuming CarParams is defined in log.capnp
            self._vin = car_params.carVin
            self._car_model = car_params.carFingerprint
        except Exception:
            pass # Handle parsing error

    if not cs.standstill and not self._is_trip_active:
      # Trip starts
      self._reset_trip_data()
      with self._lock:
        self._is_trip_active = True
        self._trip_start_time = current_time
        self._start_fuel_level = cs.fuelGauge
        print(f"Trip started at {self._trip_start_time}")

    elif cs.standstill and self._is_trip_active:
      # Trip ends
      with self._lock:
        self._is_trip_active = False
        self._trip_end_time = current_time
        self._end_fuel_level = cs.fuelGauge
        print(f"Trip ended at {self._trip_end_time}")

    if self._is_trip_active:
      with self._lock:
        # Accumulate speed samples
        self._speed_samples.append((current_time, cs.vEgo * 3.6)) # Convert m/s to kph

    self._last_car_state = cs

  def _process_live_location_kalman(self, llk):
    if not self._is_trip_active or not llk.positionGeodetic.valid:
      return

    current_lat = llk.positionGeodetic.value[0]
    current_lon = llk.positionGeodetic.value[1]

    with self._lock:
      if self._last_location:
        # Calculate distance traveled since last update
        # Using geodesic for more accurate distance over Earth's surface
        dist = geodesic(self._last_location, (current_lat, current_lon)).meters
        self._total_distance_meters += dist
      
      self._route_points.append((current_lat, current_lon))
      self._last_location = (current_lat, current_lon)

  def get_trip_data(self):
    # Only return data if a trip has ended or if there's an active trip to summarize
    if not self._trip_start_time:
      return None

    with self._lock:
      start_time = self._trip_start_time
      end_time = self._trip_end_time if not self._is_trip_active else datetime.now()
      duration_seconds = (end_time - start_time).total_seconds() if start_time else 0

      average_speed_kph = 0
      if len(self._speed_samples) > 0:
        total_speed_kph = sum(speed for _, speed in self._speed_samples)
        average_speed_kph = total_speed_kph / len(self._speed_samples)

      fuel_consumed_percent = 0
      if self._start_fuel_level is not None and self._end_fuel_level is not None:
        fuel_consumed_percent = max(0, self._start_fuel_level - self._end_fuel_level) * 100 # In percentage points

      route_geojson = None
      if len(self._route_points) > 1:
        # Simple GeoJSON LineString for the route
        route_geojson = {
            "type": "LineString",
            "coordinates": [[lon, lat] for lat, lon in self._route_points]
        }
      elif len(self._route_points) == 1:
          route_geojson = {
              "type": "Point",
              "coordinates": [self._route_points[0][1], self._route_points[0][0]]
          }

      trip_data = {
          "start_time": start_time.isoformat(),
          "end_time": end_time.isoformat(),
          "duration_seconds": round(duration_seconds, 2),
          "distance_meters": round(self._total_distance_meters, 2),
          "average_speed_kph": round(average_speed_kph, 2),
          "fuel_consumed_percentage": round(fuel_consumed_percent, 2), # Store as percentage
          "route_geojson": route_geojson,
          "vin": self._vin,
          "car_model": self._car_model,
          "is_active": self._is_trip_active # Indicate if the trip is still ongoing
      }
      return trip_data

# Global instance of the collector to be used across the application
trip_data_collector = TripDataCollector()