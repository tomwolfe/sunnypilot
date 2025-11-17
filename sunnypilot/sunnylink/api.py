import json
import os
import random
import time
from datetime import datetime, timedelta
from pathlib import Path

import jwt
import requests
from openpilot.common.api.base import BaseApi
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog

API_HOST = os.getenv('SUNNYLINK_API_HOST', 'https://stg.api.sunnypilot.ai')
UNREGISTERED_SUNNYLINK_DONGLE_ID = "UnregisteredDevice"
MAX_RETRIES = 6
CRASH_LOG_DIR = Paths.crash_log_root()

# Network resilience constants
NETWORK_TIMEOUT = 15  # seconds
MAX_BACKOFF = 300  # 5 minutes max backoff
BASE_BACKOFF = 2  # base backoff time in seconds
BACKOFF_MULTIPLIER = 1.5  # multiplier for exponential backoff


class SunnylinkApi(BaseApi):
  def __init__(self, dongle_id):
    super().__init__(dongle_id, API_HOST)
    self.user_agent = "sunnypilot-"
    self.spinner = None
    self.params = Params()
    self._last_request_time = 0
    self._error_count = 0
    self._last_error_time = 0

  def _should_retry_request(self, error_type=None):
    """Determine if a request should be retried based on error type and timing."""
    current_time = time.time()

    # Check if we had recent errors that would justify a backoff period
    if self._error_count > 0 and current_time - self._last_error_time < MAX_BACKOFF:
      # Exponential backoff based on error count
      expected_backoff = min(BASE_BACKOFF * (BACKOFF_MULTIPLIER ** min(self._error_count, 10)), MAX_BACKOFF)
      return current_time - self._last_error_time >= expected_backoff

    return True

  def _handle_request_error(self, error, endpoint):
    """Handle network request errors and update internal state."""
    self._error_count += 1
    self._last_error_time = time.time()

    cloudlog.error(f"Sunnylink API error for endpoint {endpoint}: {error}")

    # Log to crash directory as fallback
    if not os.path.exists(CRASH_LOG_DIR):
      os.makedirs(CRASH_LOG_DIR)

    with open(f'{CRASH_LOG_DIR}/sunnylink_error.txt', 'a') as f:
      f.write(f"[{datetime.now()}] sunnylink {endpoint}: {str(error)}\n")

  def api_get(self, endpoint, method='GET', timeout=10, access_token=None, json=None, **kwargs):
    """Enhanced API request with network resilience."""
    if not self.params.get_bool("SunnylinkEnabled"):
      return None

    # Check if we should proceed with the request based on recent error patterns
    if not self._should_retry_request():
      cloudlog.warning(f"Sunnylink API: Skipping request to {endpoint} due to recent errors")
      return None

    try:
      # Update the last request time
      self._last_request_time = time.time()
      response = super().api_get(endpoint, method, timeout, access_token, json, **kwargs)

      # Reset error counter on successful request
      self._error_count = 0
      return response

    except requests.exceptions.Timeout as e:
      self._handle_request_error(f"Timeout: {e}", endpoint)
      cloudlog.warning(f"Sunnylink API: Timeout for {endpoint} after {timeout}s")
      return None

    except requests.exceptions.ConnectionError as e:
      self._handle_request_error(f"ConnectionError: {e}", endpoint)
      cloudlog.warning(f"Sunnylink API: Connection error for {endpoint}: {e}")
      return None

    except requests.exceptions.RequestException as e:
      self._handle_request_error(f"RequestException: {e}", endpoint)
      cloudlog.warning(f"Sunnylink API: Request error for {endpoint}: {e}")
      return None

    except Exception as e:
      self._handle_request_error(f"Unexpected error: {e}", endpoint)
      cloudlog.error(f"Sunnylink API: Unexpected error for {endpoint}: {e}")
      return None

  def resume_queued(self, timeout=10, **kwargs):
    sunnylinkId, commaId = self._resolve_dongle_ids()
    return self.api_get(f"ws/{sunnylinkId}/resume_queued", "POST", timeout, access_token=self.get_token(), **kwargs)

  def get_token(self, payload_extra=None, expiry_hours=1):
    # Add your additional data here
    additional_data = {}
    return super()._get_token(payload_extra, expiry_hours, **additional_data)

  def _status_update(self, message):
    print(message)
    if self.spinner:
      self.spinner.update(message)
      time.sleep(0.5)

  def _resolve_dongle_ids(self):
    sunnylink_dongle_id = self.params.get("SunnylinkDongleId")
    comma_dongle_id = self.dongle_id or self.params.get("DongleId")
    return sunnylink_dongle_id, comma_dongle_id

  def _resolve_imeis(self):
    imei1, imei2 = None, None
    imei_try = 0
    while imei1 is None and imei2 is None and imei_try < MAX_RETRIES:
      try:
        imei1, imei2 = HARDWARE.get_imei(0), HARDWARE.get_imei(1)
        # Validate IMEIs are not None or empty
        if imei1 and imei2:
          break
      except Exception as e:
        self._status_update(f"Error getting imei, trying again... [{imei_try + 1}/{MAX_RETRIES}]")
        cloudlog.error(f"Sunnylink API: Error getting IMEI: {e}")
        time.sleep(1)
      imei_try += 1
    return imei1, imei2

  def _resolve_serial(self):
    try:
      serial = (self.params.get("HardwareSerial")
                or HARDWARE.get_serial())
      return serial
    except Exception as e:
      cloudlog.error(f"Sunnylink API: Error getting hardware serial: {e}")
      return None

  def register_device(self, spinner=None, timeout=60, verbose=False, max_registration_retries=3):
    self.spinner = spinner

    sunnylink_dongle_id, comma_dongle_id = self._resolve_dongle_ids()

    if comma_dongle_id is None:
      self._status_update("Comma dongle ID not found, deferring sunnylink's registration to comma's registration process.")
      cloudlog.warning("Sunnylink: Comma dongle ID not found, deferring registration")
      return None

    # Get device identifiers with better error handling
    imei1, imei2 = self._resolve_imeis()
    if not imei1 or not imei2:
      self._status_update("Failed to get valid IMEIs, cannot register device.")
      cloudlog.error("Sunnylink: Failed to get valid IMEIs, cannot register device")
      return None

    serial = self._resolve_serial()
    if not serial:
      self._status_update("Failed to get hardware serial, cannot register device.")
      cloudlog.error("Sunnylink: Failed to get hardware serial, cannot register device")
      return None

    if sunnylink_dongle_id not in (None, UNREGISTERED_SUNNYLINK_DONGLE_ID):
      cloudlog.info(f"Sunnylink already registered with ID: {sunnylink_dongle_id}")
      return sunnylink_dongle_id

    privkey_path = Path(f"{Paths.persist_root()}/comma/id_rsa")
    pubkey_path = Path(f"{Paths.persist_root()}/comma/id_rsa.pub")

    start_time = time.monotonic()
    successful_registration = False

    # Check if public key exists first
    if not pubkey_path.is_file():
      sunnylink_dongle_id = UNREGISTERED_SUNNYLINK_DONGLE_ID
      self._status_update("Public key not found, setting dongle ID to unregistered.")
      cloudlog.warning("Sunnylink: Public key not found, setting to unregistered")
    else:
      Params().put("LastSunnylinkPingTime", 0)  # Reset the last ping time to 0 if we are trying to register
      try:
        with pubkey_path.open() as f1, privkey_path.open() as f2:
          public_key = f1.read()
          private_key = f2.read()
      except Exception as e:
        self._status_update(f"Error reading key files: {str(e)}")
        cloudlog.error(f"Sunnylink: Error reading key files: {e}")
        return UNREGISTERED_SUNNYLINK_DONGLE_ID

      # Registration attempt with improved error handling and retry logic
      registration_attempts = 0
      while registration_attempts < max_registration_retries and not successful_registration:
        registration_attempts += 1

        try:
          # Generate registration token
          register_token = jwt.encode({'register': True, 'exp': datetime.utcnow() + timedelta(hours=1)},
                                    private_key, algorithm='RS256')

          if verbose or time.monotonic() - start_time < timeout / 2:
            self._status_update(f"Registering device to sunnylink... (attempt {registration_attempts}/{max_registration_retries})")
          elif time.monotonic() - start_time >= timeout / 2:
            self._status_update(f"Still registering device to sunnylink... (attempt {registration_attempts}/{max_registration_retries})")

          # Make registration request with increased timeout
          resp = self.api_get("v2/pilotauth/", method='POST', timeout=NETWORK_TIMEOUT,
                              imei=imei1, imei2=imei2, serial=serial,
                              comma_dongle_id=comma_dongle_id, public_key=public_key,
                              register_token=register_token)

          if resp is None:
            raise Exception("Unable to register device, request was None")

          if resp.status_code in (409, 412):
            # Don't retry if the public key is already in use
            key_in_use = "Public key is already in use, is your key unique? Contact your vendor for a new key."
            unsafe_key = "Public key is known to not be unique and it's unsafe. Contact your vendor for a new key."
            error_message = key_in_use if resp.status_code == 409 else unsafe_key
            cloudlog.error(f"Sunnylink registration failed permanently: {error_message}")
            raise Exception(error_message)

          if resp.status_code != 200:
            raise Exception(f"Failed to register with sunnylink. Status code: {resp.status_code}\nData\n:{resp.text}")

          # Process successful registration
          resp_data = resp.json() if hasattr(resp, 'json') and callable(getattr(resp, 'json')) else json.loads(resp.text)
          sunnylink_dongle_id = resp_data.get("device_id")

          if sunnylink_dongle_id:
            self._status_update("Device registered successfully.")
            cloudlog.info(f"Sunnylink device registered successfully with ID: {sunnylink_dongle_id}")
            successful_registration = True
            break
          else:
            raise Exception("Registration response missing device_id")

        except Exception as e:
          cloudlog.error(f"Sunnylink registration attempt {registration_attempts} failed: {str(e)}")

          if registration_attempts >= max_registration_retries:
            self._status_update(f"Failed all {max_registration_retries} registration attempts.")
            cloudlog.error(f"Sunnylink: Failed all {max_registration_retries} registration attempts")
          else:
            # Calculate backoff time with jitter
            backoff = min(BASE_BACKOFF * (BACKOFF_MULTIPLIER ** (registration_attempts - 1)) * (0.5 + random.random()), MAX_BACKOFF)
            self._status_update(f"Waiting {backoff:.1f}s before retry...")
            time.sleep(backoff)

        # Check timeout
        if time.monotonic() - start_time > timeout:
          self._status_update(f"Giving up on sunnylink's registration after {timeout}s. Will retry on next boot.")
          cloudlog.info(f"Sunnylink: Gave up on registration after {timeout}s")
          break

    # Always update the parameter, even if registration failed
    final_dongle_id = sunnylink_dongle_id or UNREGISTERED_SUNNYLINK_DONGLE_ID
    self.params.put("SunnylinkDongleId", final_dongle_id)

    # Set the last ping time to the current time since we were just talking to the API
    last_ping = int((time.monotonic() if successful_registration else start_time) * 1e9)
    Params().put("LastSunnylinkPingTime", last_ping)

    # Disable sunnylink if registration was not successful
    if not successful_registration:
      Params().put_bool("SunnylinkEnabled", False)
      cloudlog.warning("Sunnylink: Registration failed, disabling sunnylink")

    self.spinner = None
    return final_dongle_id
