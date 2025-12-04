import asyncio
import time
import platform

from cereal import messaging
from openpilot.common.realtime import DT_MDL, DT_DMON

# Import av library conditionally since it's not available on macOS
if platform.system() != "Darwin":
  import av
from teleoprtc.tracks import TiciVideoStreamTrack


class LiveStreamVideoStreamTrack(TiciVideoStreamTrack):
  camera_to_sock_mapping = {
    "driver": "livestreamDriverEncodeData",
    "wideRoad": "livestreamWideRoadEncodeData",
    "road": "livestreamRoadEncodeData",
  }

  def __init__(self, camera_type: str):
    dt = DT_DMON if camera_type == "driver" else DT_MDL
    super().__init__(camera_type, dt)

    self._sock = messaging.sub_sock(self.camera_to_sock_mapping[camera_type], conflate=True)
    self._pts = 0
    self._t0_ns = time.monotonic_ns()

  async def recv(self):
    while True:
      msg = messaging.recv_one_or_none(self._sock)
      if msg is not None:
        break
      await asyncio.sleep(0.005)

    evta = getattr(msg, msg.which())

    # Only create av packet if av is available (not on macOS)
    if platform.system() != "Darwin":
      packet = av.Packet(evta.header + evta.data)
      packet.time_base = self._time_base

      self._pts = ((time.monotonic_ns() - self._t0_ns) * self._clock_rate) // 1_000_000_000
      packet.pts = self._pts
      self.log_debug("track sending frame %d", self._pts)

      return packet
    else:
      # On macOS where av is not available, raise an exception or return appropriate error
      raise NotImplementedError("Video streaming not supported on macOS due to av library incompatibility")

  def codec_preference(self) -> str | None:
    return "H264"
