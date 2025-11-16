"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import asyncio
import os
import time
from typing import Dict, Optional

import aiohttp
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware.hw import Paths

from cereal import messaging, custom
from sunnypilot.models.fetcher import ModelFetcher
from sunnypilot.models.helpers import verify_file, get_active_bundle


class ModelManagerSP:
  """Manages model downloads and status reporting"""

  def __init__(self):
    self.params = Params()
    self.model_fetcher = ModelFetcher(self.params)
    self.pm = messaging.PubMaster(["modelManagerSP"])
    self.available_models: list[custom.ModelManagerSP.ModelBundle] = []
    self.selected_bundle: custom.ModelManagerSP.ModelBundle = None
    self.active_bundle: custom.ModelManagerSP.ModelBundle = get_active_bundle(self.params)
    # Optimized chunk size for better performance
    self._chunk_size = 256 * 1000  # 256 KB chunks for better throughput
    self._download_start_times: dict[str, float] = {}  # Track start time per model
    # Track bandwidth for better ETA calculation
    self._download_speed_history: Dict[str, list] = {}
    # Bandwidth management
    self._max_concurrent_downloads = 2  # Limit concurrent downloads to prevent resource exhaustion
    try:
      self._throttle_downloads = self.params.get_bool("ModelManager_ThrottleDownloads")
    except Exception:
      self._throttle_downloads = False  # Default to False if parameter doesn't exist

  def _calculate_eta(self, filename: str, progress: float, bytes_downloaded: int, elapsed_time: float) -> int:
    """Calculate ETA based on download speed with historical data"""
    if filename not in self._download_start_times or progress <= 0:
      return 120  # Default ETA for new downloads

    # Calculate current speed
    if elapsed_time > 0:
      current_speed = bytes_downloaded / elapsed_time  # bytes per second

      # Maintain speed history for better ETA estimation
      if filename not in self._download_speed_history:
        self._download_speed_history[filename] = []

      # Add current speed to history (limit to last 10 measurements)
      self._download_speed_history[filename].append(current_speed)
      self._download_speed_history[filename] = self._download_speed_history[filename][-10:]

      # Calculate average speed from history
      avg_speed = sum(self._download_speed_history[filename]) / len(self._download_speed_history[filename])
    else:
      avg_speed = 1000000  # 1 MB/s default if no data

    if avg_speed <= 0:
      return 120  # Default if speed is 0

    # Calculate remaining bytes and time
    remaining_bytes = bytes_downloaded * (100 - progress) / progress if progress < 100 else 0
    estimated_time = remaining_bytes / avg_speed

    return max(1, int(estimated_time))  # Return at least 1 second

  async def _download_file(self, url: str, path: str, model) -> None:
    """Downloads a file with progress tracking, bandwidth management and enhanced retry logic"""
    start_time = time.monotonic()
    self._download_start_times[model.fileName] = start_time

    # Add advanced headers for better server compatibility
    headers = {
        'User-Agent': 'sunnypilot-model-downloader/2.0',
        'Accept': '*/*',
        'Accept-Encoding': 'identity',  # Avoid compression for proper progress tracking
        'Connection': 'keep-alive',
        'Range': None,  # Will be set if resuming partial downloads
    }

    # Check for partial downloads that can be resumed
    resume_pos = 0
    if os.path.exists(path):
      resume_pos = os.path.getsize(path)
      if resume_pos > 0:
        headers['Range'] = f'bytes={resume_pos}-'
        cloudlog.info(f"Resuming download for {model.fileName} from byte {resume_pos}")

    # Set appropriate timeout based on file size (if known)
    timeout_duration = 300  # 5 minutes default
    if self._throttle_downloads:
      timeout_duration = 600  # 10 minutes for throttled downloads

    max_retries = 5  # Increased retries for network resilience
    attempt = 0
    bytes_downloaded = resume_pos  # Start with already downloaded bytes

    while attempt < max_retries:
      try:
        async with aiohttp.ClientSession(
          timeout=aiohttp.ClientTimeout(total=timeout_duration),
          connector=aiohttp.TCPConnector(limit=1)  # Limit connections per session
        ) as session:
          async with session.get(url, headers=headers, raise_for_status=True) as response:
            # Handle 206 Partial Content (resume) or 200 OK (fresh download)
            if response.status in [200, 206]:
              # Get total size - account for range requests
              total_size_header = response.headers.get("content-length")
              if total_size_header:
                total_size = int(total_size_header)
                # For range requests, add the resume position to get true total
                if resume_pos > 0:
                  total_size += resume_pos
              else:
                total_size = 0  # Unknown size

              # Create directory if it doesn't exist
              os.makedirs(os.path.dirname(path), exist_ok=True)

              # Open file in append mode if resuming, write mode if fresh
              mode = 'ab' if resume_pos > 0 else 'wb'
              with open(path, mode) as f:
                # Adjust chunk size based on connection quality
                adaptive_chunk_size = self._chunk_size
                if self._throttle_downloads:
                  adaptive_chunk_size = 64 * 1000  # 64KB for throttled downloads

                async for chunk in response.content.iter_chunked(adaptive_chunk_size):
                  f.write(chunk)
                  bytes_downloaded += len(chunk)

                  if total_size > 0:
                    progress = (bytes_downloaded / total_size) * 100
                    elapsed_time = time.monotonic() - start_time

                    model.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.downloading
                    model.downloadProgress.progress = progress
                    model.downloadProgress.eta = self._calculate_eta(
                      model.fileName, progress, bytes_downloaded, elapsed_time
                    )
                    # Report status more frequently during download for better UI responsiveness
                    if int(progress) % 5 == 0 or progress >= 99.9:  # Update every 5% or on completion
                      self._report_status()

              # Clean up start time after download completes
              del self._download_start_times[model.fileName]
              # Clean up speed history
              if model.fileName in self._download_speed_history:
                del self._download_speed_history[model.fileName]
              return  # Success, exit the retry loop
            else:
              raise aiohttp.ClientResponseError(
                request_info=response.request_info,
                history=response.history,
                status=response.status,
                message=f"Unexpected response status: {response.status}"
              )

      except (aiohttp.ClientResponseError, aiohttp.ClientConnectorError, TimeoutError) as e:
        attempt += 1
        cloudlog.warning(f"Download attempt {attempt}/{max_retries} failed for {model.fileName}: {type(e).__name__} - {str(e)}")

        if attempt >= max_retries:
          raise  # All retries exhausted

        # Exponential backoff with jitter
        backoff_time = min(2 ** attempt * (0.5 + (time.time() % 0.5)), 60)  # Max 60 seconds
        cloudlog.info(f"Waiting {backoff_time:.1f}s before retrying download...")
        await asyncio.sleep(backoff_time)

        # For resumeable downloads, reset resume position if needed
        if os.path.exists(path) and resume_pos > 0:
          # Verify the downloaded part is valid before resuming
          expected_hash = getattr(model, 'expected_hash', None)
          if expected_hash and await verify_file(path, expected_hash):
            # File is valid up to current position, continue from here
            resume_pos = os.path.getsize(path)
            headers['Range'] = f'bytes={resume_pos}-'
          else:
            # File is corrupted, start over
            cloudlog.warning(f"Downloaded part of {model.fileName} is corrupted, restarting...")
            os.remove(path)
            resume_pos = 0
            del headers['Range']

      except Exception as e:
        cloudlog.error(f"Unexpected error downloading {model.fileName}: {str(e)}")
        raise

  async def _process_artifact(self, artifact, destination_path: str) -> None:
    """Processes a single model download including verification"""
    if not artifact.downloadUri.uri:
      cloudlog.warning(f"No download URI provided for artifact: {artifact.fileName}")
      return None

    url = artifact.downloadUri.uri
    expected_hash = artifact.downloadUri.sha256
    filename = artifact.fileName
    full_path = os.path.join(destination_path, filename)

    # Store expected hash in artifact for verification during resume
    artifact.expected_hash = expected_hash

    try:
      # Check existing file with verification
      if os.path.exists(full_path):
        if await verify_file(full_path, expected_hash):
          cloudlog.info(f"Using cached model file: {filename}")
          artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.cached
          artifact.downloadProgress.progress = 100
          artifact.downloadProgress.eta = 0
          self._report_status()
          return

        # If file exists but hash doesn't match, remove it and re-download
        cloudlog.info(f"File exists but hash mismatch, removing: {filename}")
        os.remove(full_path)

      # Download and verify
      cloudlog.info(f"Starting download for {filename} from {url} (size: ~{(artifact.downloadUri.sizeBytes or 0) / (1024*1024):.1f}MB)")
      await self._download_file(url, full_path, artifact)

      # Final verification after download completion
      if not await verify_file(full_path, expected_hash):
        raise ValueError(f"Final hash validation failed for {filename}")

      cloudlog.info(f"Successfully downloaded and verified {filename}")
      artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.downloaded
      artifact.downloadProgress.eta = 0
      self._report_status()

    except Exception as e:
      cloudlog.error(f"Error processing artifact {filename}: {str(e)}")
      if os.path.exists(full_path):
        try:
          os.remove(full_path)
          cloudlog.debug(f"Removed incomplete/corrupted download file: {full_path}")
        except OSError as rm_error:
          cloudlog.warning(f"Failed to remove incomplete file {full_path}: {str(rm_error)}")

      artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.failed
      artifact.downloadProgress.eta = 0
      if self.selected_bundle:  # Only update bundle status if we're currently downloading
        self.selected_bundle.status = custom.ModelManagerSP.DownloadStatus.failed
      self._report_status()
      # Clean up start time if it exists
      self._download_start_times.pop(artifact.fileName, None)
      # Clean up speed history
      if artifact.fileName in self._download_speed_history:
        del self._download_speed_history[artifact.fileName]
      raise

  async def _process_model(self, model, destination_path: str) -> None:
    """Processes a single model download including verification"""
    # Process metadata first to fail fast if metadata download fails
    await self._process_artifact(model.metadata, destination_path)
    # Then process main model artifact
    await self._process_artifact(model.artifact, destination_path)

  def _report_status(self) -> None:
    """Reports current status through messaging system"""
    try:
      msg = messaging.new_message('modelManagerSP', valid=True)
      model_manager_state = msg.modelManagerSP
      if self.selected_bundle:
        model_manager_state.selectedBundle = self.selected_bundle

      if self.active_bundle:
        model_manager_state.activeBundle = self.active_bundle

      model_manager_state.availableBundles = self.available_models
      self.pm.send('modelManagerSP', msg)
    except Exception as e:
      cloudlog.error(f"Error reporting model manager status: {str(e)}")

  async def _download_bundle(self, model_bundle: custom.ModelManagerSP.ModelBundle, destination_path: str) -> None:
    """Downloads all models in a bundle with optimized concurrency"""
    self.selected_bundle = model_bundle
    self.selected_bundle.status = custom.ModelManagerSP.DownloadStatus.downloading
    os.makedirs(destination_path, exist_ok=True)

    try:
      # Use semaphore to limit concurrent downloads
      semaphore = asyncio.Semaphore(self._max_concurrent_downloads)

      async def download_with_semaphore(model):
        async with semaphore:
          return await self._process_model(model, destination_path)

      # Create tasks with semaphore control
      tasks = [download_with_semaphore(model) for model in self.selected_bundle.models]
      await asyncio.gather(*tasks, return_exceptions=True)

      # Check if any tasks failed
      failed_downloads = [task for task in tasks if isinstance(task, Exception)]
      if failed_downloads:
        cloudlog.error(f"Some model downloads failed in bundle {model_bundle.displayName}")
        raise Exception(f"Bundle download failed: {len(failed_downloads)} models failed")

      self.active_bundle = self.selected_bundle
      self.active_bundle.status = custom.ModelManagerSP.DownloadStatus.downloaded
      # Store bundle info with proper serialization
      try:
        bundle_dict = self.active_bundle.to_dict()
        self.params.put("ModelManager_ActiveBundle", bundle_dict)
      except Exception as e:
        cloudlog.error(f"Failed to save active bundle info: {str(e)}")
      self.selected_bundle = None

    except Exception as e:
      cloudlog.error(f"Bundle download failed for {model_bundle.displayName}: {str(e)}")
      self.selected_bundle.status = custom.ModelManagerSP.DownloadStatus.failed
      raise

    finally:
      self._report_status()

  def download(self, model_bundle: custom.ModelManagerSP.ModelBundle, destination_path: str) -> None:
    """Main entry point for downloading a model bundle"""
    cloudlog.info(f"Starting download of model bundle: {model_bundle.displayName} (index: {model_bundle.index}) "
                 f"with {len(model_bundle.models)} models")
    try:
      asyncio.run(self._download_bundle(model_bundle, destination_path))
      cloudlog.info(f"Successfully completed download of model bundle: {model_bundle.displayName}")
    except Exception as e:
      cloudlog.error(f"Failed to download model bundle {model_bundle.displayName}: {str(e)}")
      raise

  def main_thread(self) -> None:
    """Main thread for model management"""
    rk = Ratekeeper(1, print_delay_threshold=None)
    last_model_count = -1  # Track for logging changes
    last_params_update = 0  # Track time for params updates
    param_update_interval = 10  # Update params every 10 seconds instead of every iteration

    while True:
      try:
        # Update params periodically instead of every loop
        current_time = time.monotonic()
        if current_time - last_params_update >= param_update_interval:
          try:
            self._throttle_downloads = self.params.get_bool("ModelManager_ThrottleDownloads")
          except Exception:
            self._throttle_downloads = False  # Default to False if parameter doesn't exist
          last_params_update = current_time

        self.available_models = self.model_fetcher.get_available_bundles()
        self.active_bundle = get_active_bundle(self.params)

        # Log model count changes to help with debugging
        if len(self.available_models) != last_model_count:
          active_bundle_name = self.active_bundle.displayName if self.active_bundle else 'default'
          cloudlog.info(f"Model manager: {len(self.available_models)} available model bundles, active: {active_bundle_name}")
          last_model_count = len(self.available_models)

        if index_to_download := self.params.get("ModelManager_DownloadIndex"):
          if model_to_download := next((model for model in self.available_models if model.index == index_to_download), None):
            cloudlog.info(f"Initiating download for model: {model_to_download.displayName}")
            try:
              self.download(model_to_download, Paths.model_root())
              cloudlog.info(f"Download completed successfully for model: {model_to_download.displayName}")
              # Clear download success flag if there's one
              self.params.remove("ModelManager_DownloadSuccess")  # Clear previous success
              self.params.put_bool("ModelManager_DownloadSuccess", True)  # Mark success
            except Exception as e:
              cloudlog.error(f"Download failed for model {model_to_download.displayName}: {str(e)}")
              # Store error info for UI to display
              error_msg = f"Download failed: {str(e)[:250]}"  # Limit length
              self.params.put("ModelManager_LastError", error_msg)
              self.params.put("ModelManager_LastErrorTime", str(int(time.time())))
            finally:
              self.params.remove("ModelManager_DownloadIndex")
          else:
            cloudlog.warning(f"Requested model index {index_to_download} not found in available models")
            error_msg = f"Model index {index_to_download} not found"
            self.params.put("ModelManager_LastError", error_msg)
            self.params.put("ModelManager_LastErrorTime", str(int(time.time())))

        if self.params.get("ModelManager_ClearCache"):
            cloudlog.info("Clearing model cache")
            self.clear_model_cache()
            self.params.remove("ModelManager_ClearCache")

        self._report_status()
        rk.keep_time()

      except Exception as e:
        cloudlog.exception(f"Error in main thread: {str(e)}")
        rk.keep_time()

  def clear_model_cache(self) -> None:
    """
    Clears the model cache directory of all files except those in the active model bundle.
    """
    try:
      # Get list of files used by active model bundle
      active_files = set()
      if self.active_bundle is not None:  # When the default model is active
        for model in self.active_bundle.models:
          if hasattr(model, 'artifact') and model.artifact.fileName:
            active_files.add(model.artifact.fileName)
          if hasattr(model, 'metadata') and model.metadata.fileName:
            active_files.add(model.metadata.fileName)

      # Remove all files except active ones
      model_dir = Paths.model_root()
      files_removed = 0
      for filename in os.listdir(model_dir):
        if filename not in active_files:
          file_path = os.path.join(model_dir, filename)
          if os.path.isfile(file_path):
            os.remove(file_path)
            files_removed += 1

      cloudlog.info(f"Model cache cleared: {files_removed} files removed, keeping {len(active_files)} active model files")

    except Exception as e:
      cloudlog.exception(f"Error clearing model cache: {str(e)}")

def main():
  ModelManagerSP().main_thread()


if __name__ == "__main__":
  main()
