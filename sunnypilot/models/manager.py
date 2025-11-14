"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import asyncio
import os
import time

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
    self._chunk_size = 128 * 1000  # 128 KB chunks
    self._download_start_times: dict[str, float] = {}  # Track start time per model

  def _calculate_eta(self, filename: str, progress: float) -> int:
    """Calculate ETA based on elapsed time and current progress"""
    if filename not in self._download_start_times or progress <= 0:
      return 60  # Default ETA for new downloads

    elapsed_time = time.monotonic() - self._download_start_times[filename]
    if elapsed_time <= 0:
      return 60

    # If we're at X% after Y seconds, we can estimate total time as (Y / X) * 100
    total_estimated_time = (elapsed_time / progress) * 100
    eta = total_estimated_time - elapsed_time

    return max(1, int(eta))  # Return at least 1 second if download is ongoing

  async def _download_file(self, url: str, path: str, model) -> None:
    """Downloads a file with progress tracking and retry logic"""
    self._download_start_times[model.fileName] = time.monotonic()

    # Add headers to make request more robust
    headers = {
        'User-Agent': 'sunnypilot-model-downloader/1.0',
        'Accept': '*/*',
        'Accept-Encoding': 'identity',  # Avoid compression for proper progress tracking
        'Connection': 'keep-alive'
    }

    max_retries = 3
    for attempt in range(max_retries):
      try:
        async with aiohttp.ClientSession() as session:
          async with session.get(url, headers=headers, timeout=aiohttp.ClientTimeout(total=300)) as response:  # 5 minute timeout per file
            response.raise_for_status()
            total_size = int(response.headers.get("content-length", 0))
            bytes_downloaded = 0

            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(path), exist_ok=True)

            with open(path, 'wb') as f:
              async for chunk in response.content.iter_chunked(self._chunk_size):  # type: bytes
                f.write(chunk)
                bytes_downloaded += len(chunk)

                if total_size > 0:
                  progress = (bytes_downloaded / total_size) * 100
                  model.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.downloading
                  model.downloadProgress.progress = progress
                  model.downloadProgress.eta = self._calculate_eta(model.fileName, progress)
                  self._report_status()

            # Clean up start time after download completes
            del self._download_start_times[model.fileName]
            return  # Success, exit the retry loop

      except asyncio.TimeoutError:
        cloudlog.warning(f"Download timeout for {model.fileName} (attempt {attempt + 1}/{max_retries})")
        if attempt == max_retries - 1:  # Last attempt
          raise
      except aiohttp.ClientResponseError as e:
        cloudlog.warning(f"HTTP error downloading {model.fileName}: {e.status} {e.message} (attempt {attempt + 1}/{max_retries})")
        if e.status in [401, 403, 404]:  # Don't retry on these errors
          raise
        if attempt == max_retries - 1:  # Last attempt
          raise
      except aiohttp.ClientError as e:
        cloudlog.warning(f"Network error downloading {model.fileName}: {str(e)} (attempt {attempt + 1}/{max_retries})")
        if attempt == max_retries - 1:  # Last attempt
          raise
      except Exception as e:
        cloudlog.warning(f"Unexpected error downloading {model.fileName}: {str(e)} (attempt {attempt + 1}/{max_retries})")
        if attempt == max_retries - 1:  # Last attempt
          raise
      # Wait before retrying (with exponential backoff)
      if attempt < max_retries - 1:
        await asyncio.sleep(min(2 ** attempt, 10))  # Max 10 seconds between retries

  async def _process_artifact(self, artifact, destination_path: str) -> None:
    """Processes a single model download including verification"""
    if not artifact.downloadUri.uri:
      cloudlog.warning(f"No download URI provided for artifact: {artifact.fileName}")
      return None

    url = artifact.downloadUri.uri
    expected_hash = artifact.downloadUri.sha256
    filename = artifact.fileName
    full_path = os.path.join(destination_path, filename)

    try:
      # Check existing file
      if os.path.exists(full_path) and await verify_file(full_path, expected_hash):
        cloudlog.info(f"Using cached model file: {filename}")
        artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.cached
        artifact.downloadProgress.progress = 100
        artifact.downloadProgress.eta = 0
        self._report_status()
        return

      # Download and verify
      cloudlog.info(f"Starting download for {filename} from {url}")
      await self._download_file(url, full_path, artifact)
      if not await verify_file(full_path, expected_hash):
        raise ValueError(f"Hash validation failed for {filename}")

      cloudlog.info(f"Successfully downloaded and verified {filename}")
      artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.downloaded
      artifact.downloadProgress.eta = 0
      self._report_status()

    except Exception as e:
      cloudlog.error(f"Error processing artifact {filename}: {str(e)}")
      if os.path.exists(full_path):
        try:
          os.remove(full_path)
          cloudlog.debug(f"Removed incomplete download file: {full_path}")
        except OSError as rm_error:
          cloudlog.warning(f"Failed to remove incomplete file {full_path}: {str(rm_error)}")

      artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.failed
      artifact.downloadProgress.eta = 0
      if self.selected_bundle:  # Only update bundle status if we're currently downloading
        self.selected_bundle.status = custom.ModelManagerSP.DownloadStatus.failed
      self._report_status()
      # Clean up start time if it exists
      self._download_start_times.pop(artifact.fileName, None)
      raise

  async def _process_model(self, model, destination_path: str) -> None:
    """Processes a single model download including verification"""
    model_artifact = model.artifact
    metadata_artifact = model.metadata

    await self._process_artifact(metadata_artifact, destination_path)
    await self._process_artifact(model_artifact, destination_path)

  def _report_status(self) -> None:
    """Reports current status through messaging system"""
    msg = messaging.new_message('modelManagerSP', valid=True)
    model_manager_state = msg.modelManagerSP
    if self.selected_bundle:
      model_manager_state.selectedBundle = self.selected_bundle

    if self.active_bundle:
      model_manager_state.activeBundle = self.active_bundle

    model_manager_state.availableBundles = self.available_models
    self.pm.send('modelManagerSP', msg)

  async def _download_bundle(self, model_bundle: custom.ModelManagerSP.ModelBundle, destination_path: str) -> None:
    """Downloads all models in a bundle"""
    self.selected_bundle = model_bundle
    self.selected_bundle.status = custom.ModelManagerSP.DownloadStatus.downloading
    os.makedirs(destination_path, exist_ok=True)

    try:
      tasks = [self._process_model(model, destination_path) for model in self.selected_bundle.models]
      await asyncio.gather(*tasks)
      self.active_bundle = self.selected_bundle
      self.active_bundle.status = custom.ModelManagerSP.DownloadStatus.downloaded
      self.params.put("ModelManager_ActiveBundle", self.active_bundle.to_dict())
      self.selected_bundle = None

    except Exception:
      self.selected_bundle.status = custom.ModelManagerSP.DownloadStatus.failed
      raise

    finally:
      self._report_status()

  def download(self, model_bundle: custom.ModelManagerSP.ModelBundle, destination_path: str) -> None:
    """Main entry point for downloading a model bundle"""
    cloudlog.info(f"Starting download of model bundle: {model_bundle.displayName} (index: {model_bundle.index})")
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

    while True:
      try:
        self.available_models = self.model_fetcher.get_available_bundles()
        self.active_bundle = get_active_bundle(self.params)

        # Log model count changes to help with debugging
        if len(self.available_models) != last_model_count:
          cloudlog.info(f"Model manager: {len(self.available_models)} available model bundles, active: {self.active_bundle.displayName if self.active_bundle else 'default'}")
          last_model_count = len(self.available_models)

        if index_to_download := self.params.get("ModelManager_DownloadIndex"):
          if model_to_download := next((model for model in self.available_models if model.index == index_to_download), None):
            cloudlog.info(f"Initiating download for model: {model_to_download.displayName}")
            try:
              self.download(model_to_download, Paths.model_root())
              cloudlog.info(f"Download completed successfully for model: {model_to_download.displayName}")
            except Exception as e:
              cloudlog.error(f"Download failed for model {model_to_download.displayName}: {str(e)}")
              # Store error info for UI to display
              self.params.put("ModelManager_LastError", f"Download failed: {str(e)[:250]}")  # Limit length
            finally:
              self.params.remove("ModelManager_DownloadIndex")
          else:
            cloudlog.warning(f"Requested model index {index_to_download} not found in available models")
            self.params.put("ModelManager_LastError", f"Model index {index_to_download} not found")

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

    # Get list of files used by active model bundle
    active_files = []
    if self.active_bundle is not None: # When the default model is active
      for model in self.active_bundle.models:
        if hasattr(model, 'artifact') and model.artifact.fileName:
          active_files.append(model.artifact.fileName)
        if hasattr(model, 'metadata') and model.metadata.fileName:
          active_files.append(model.metadata.fileName)

    # Remove all files except active ones
    model_dir = Paths.model_root()
    try:
      for filename in os.listdir(model_dir):
        if filename not in active_files:
          file_path = os.path.join(model_dir, filename)
          if os.path.isfile(file_path):
            os.remove(file_path)
      cloudlog.info("Model cache cleared, keeping active model files")
    except Exception as e:
      cloudlog.exception(f"Error clearing model cache: {str(e)}")

def main():
  ModelManagerSP().main_thread()


if __name__ == "__main__":
  main()
