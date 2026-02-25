import pytest
import asyncio
import json
# for aiortc and its dependencies
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=RuntimeWarning) # TODO: remove this when google-crc32c publish a python3.12 wheel

from openpilot.system.webrtc.webrtcd import get_stream

import aiortc
from teleoprtc import WebRTCOfferBuilder
from parameterized import parameterized_class


@parameterized_class(("in_services", "out_services"), [
  (["testJoystick"], ["carState"]),
  ([], ["carState"]),
  (["testJoystick"], []),
  ([], []),
])
@pytest.mark.asyncio
class TestWebrtcdProc:
  async def assertCompletesWithTimeout(self, awaitable, timeout=1):
    try:
      async with asyncio.timeout(timeout):
        await awaitable
    except TimeoutError:
      pytest.fail("Timeout while waiting for awaitable to complete")

  async def test_webrtcd(self, mocker):
    mock_request = mocker.MagicMock()
    session = None
    try:
      async def connect(offer):
        nonlocal session
        body = {'sdp': offer.sdp, 'cameras': offer.video, 'bridge_services_in': self.in_services, 'bridge_services_out': self.out_services}
        mock_request.json.side_effect = mocker.AsyncMock(return_value=body)
        response = await get_stream(mock_request)
        response_json = json.loads(response.text)
        session = mock_request.app["streams"].__setitem__.call_args.args[1]
        return aiortc.RTCSessionDescription(**response_json)

      builder = WebRTCOfferBuilder(connect)
      builder.offer_to_receive_video_stream("road")
      builder.offer_to_receive_audio_stream()
      if len(self.in_services) > 0 or len(self.out_services) > 0:
        builder.add_messaging()

      stream = builder.stream()

      await self.assertCompletesWithTimeout(stream.start())
      await self.assertCompletesWithTimeout(stream.wait_for_connection())

      assert stream.has_incoming_video_track("road")
      assert stream.has_incoming_audio_track()
      assert stream.has_messaging_channel() == (len(self.in_services) > 0 or len(self.out_services) > 0)

      video_track, audio_track = stream.get_incoming_video_track("road"), stream.get_incoming_audio_track()
      await self.assertCompletesWithTimeout(video_track.recv())
      await self.assertCompletesWithTimeout(audio_track.recv())

      await self.assertCompletesWithTimeout(stream.stop())
    finally:
      # Ensure session cleanup is robust even if test fails early
      if session is not None:
        try:
          async with asyncio.timeout(5):
            await session.post_run_cleanup()
        except (asyncio.TimeoutError, Exception):
          pass
        
      # Cleanup streams from mock request
      if hasattr(mock_request.app, "streams"):
        for s in list(mock_request.app["streams"].values()):
          try:
            s.stop()
          except Exception:
            pass

