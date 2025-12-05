import asyncio
import io
import platform
from typing import TYPE_CHECKING, Any

import numpy as np
import pyaudio

# Import aiortc and av for type checking purposes
if TYPE_CHECKING:
    import aiortc
    import av


def _check_platform_compatibility():
    """Helper function to check if platform supports WebRTC audio functionality."""
    if platform.system() == "Darwin":
        raise NotImplementedError("Audio streaming not supported on macOS due to av library incompatibility")


class AudioInputStreamTrack:
    """Platform-agnostic audio input stream track."""

    def __init__(self, audio_format: int = pyaudio.paInt16, rate: int = 16000, channels: int = 1, packet_time: float = 0.020, device_index: int = None):
        _check_platform_compatibility()  # This will raise an error on macOS

        if platform.system() != "Darwin":
            # Import required modules only when needed and on compatible platforms
            import aiortc
            import av
            import pyaudio
            import numpy as np

            # Set up mapping (using the same approach as before)
            self.PYAUDIO_TO_AV_FORMAT_MAP = {
                pyaudio.paUInt8: 'u8',
                pyaudio.paInt16: 's16',
                pyaudio.paInt24: 's24',
                pyaudio.paInt32: 's32',
                pyaudio.paFloat32: 'flt',
            }

            self.p = pyaudio.PyAudio()
            chunk_size = int(packet_time * rate)
            self.stream = self.p.open(format=audio_format, channels=channels, rate=rate,
                                      frames_per_buffer=chunk_size, input=True, input_device_index=device_index)
            self.format = audio_format
            self.rate = rate
            self.channels = channels
            self.packet_time = packet_time
            self.chunk_size = chunk_size
            self.pts = 0

    async def recv(self):
        _check_platform_compatibility()  # This will raise an error on macOS

        # Import av only when needed and on compatible platforms
        import av

        mic_data = self.stream.read(self.chunk_size)
        mic_array = np.frombuffer(mic_data, dtype=np.int16)
        mic_array = np.expand_dims(mic_array, axis=0)
        layout = 'stereo' if self.channels > 1 else 'mono'
        frame = av.AudioFrame.from_ndarray(mic_array, format=self.PYAUDIO_TO_AV_FORMAT_MAP[self.format], layout=layout)
        frame.rate = self.rate
        frame.pts = self.pts
        self.pts += frame.samples

        return frame


class AudioOutputSpeaker:
    """Platform-agnostic audio output speaker."""

    def __init__(self, audio_format: int = pyaudio.paInt16, rate: int = 48000, channels: int = 2, packet_time: float = 0.2, device_index: int = None):
        _check_platform_compatibility()  # This will raise an error on macOS

        if platform.system() != "Darwin":
            import asyncio
            import io

            chunk_size = int(packet_time * rate)
            self.p = pyaudio.PyAudio()
            self.buffer = io.BytesIO()
            self.channels = channels
            self.stream = self.p.open(
                format=audio_format,
                channels=channels,
                rate=rate,
                frames_per_buffer=chunk_size,
                output=True,
                output_device_index=device_index,
                stream_callback=self.__pyaudio_callback,
            )
            self.tracks_and_tasks: list[tuple[Any, asyncio.Task | None]] = []

    def __pyaudio_callback(self, in_data, frame_count, time_info, status):
        _check_platform_compatibility()  # This will raise an error on macOS

        if self.buffer.getbuffer().nbytes < frame_count * self.channels * 2:
            buff = b'\x00\x00' * frame_count * self.channels
        elif self.buffer.getbuffer().nbytes > 115200:  # 3x the usual read size
            self.buffer.seek(0)
            buff = self.buffer.read(frame_count * self.channels * 4)
            buff = buff[: frame_count * self.channels * 2]
            self.buffer.seek(2)
        else:
            self.buffer.seek(0)
            buff = self.buffer.read(frame_count * self.channels * 2)
            self.buffer.seek(2)
        return (buff, pyaudio.paContinue)

    async def __consume(self, track):
        _check_platform_compatibility()  # This will raise an error on macOS

        while True:
            try:
                frame = await track.recv()
            except:  # Using generic exception as aiortc.MediaStreamError may not be available
                return

            self.buffer.write(bytes(frame.planes[0]))

    def hasTrack(self, track: Any) -> bool:
        _check_platform_compatibility()  # This will raise an error on macOS
        return any(t == track for t, _ in self.tracks_and_tasks)

    def addTrack(self, track: Any):
        _check_platform_compatibility()  # This will raise an error on macOS
        if not self.hasTrack(track):
            self.tracks_and_tasks.append((track, None))

    def start(self):
        _check_platform_compatibility()  # This will raise an error on macOS
        for index, (track, task) in enumerate(self.tracks_and_tasks):
            if task is None:
                self.tracks_and_tasks[index] = (track, asyncio.create_task(self.__consume(track)))

    def stop(self):
        _check_platform_compatibility()  # This will raise an error on macOS
        for _, task in self.tracks_and_tasks:
            if task is not None:
                task.cancel()

        self.tracks_and_tasks = []
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
