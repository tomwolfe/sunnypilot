import numpy as np
import os
from openpilot.common.params import Params

# conversions
class CV:
  # Speed
  MPH_TO_KPH = 1.609344
  KPH_TO_MPH = 1. / MPH_TO_KPH
  MS_TO_KPH = 3.6
  KPH_TO_MS = 1. / MS_TO_KPH
  MS_TO_MPH = MS_TO_KPH * KPH_TO_MPH
  MPH_TO_MS = MPH_TO_KPH * KPH_TO_MS
  MS_TO_KNOTS = 1.9438
  KNOTS_TO_MS = 1. / MS_TO_KNOTS

  # Angle
  DEG_TO_RAD = np.pi / 180.
  RAD_TO_DEG = 1. / DEG_TO_RAD

  # Mass
  LB_TO_KG = 0.453592


ACCELERATION_DUE_TO_GRAVITY = 9.81  # m/s^2

# sunnypilot trip data
DEFAULT_TRIP_DATA_PATH = "/data/media/0/sunnypilot/trip_data/"
TRIP_DATA_PATH_PARAM_KEY = "CustomTripDataPath"
TRIP_DATA_RETENTION_COUNT_PARAM_KEY = "TripDataRetentionCount"
DEFAULT_TRIP_DATA_RETENTION_COUNT = 50 # Keep last 50 trip files

def get_trip_data_path():
  params = Params()
  custom_path = params.get(TRIP_DATA_PATH_PARAM_KEY, encoding='utf-8')
  if custom_path and os.path.isdir(custom_path): # Only use custom path if it exists and is a directory
    return custom_path
  return DEFAULT_TRIP_DATA_PATH

TRIP_DATA_PATH = get_trip_data_path()

