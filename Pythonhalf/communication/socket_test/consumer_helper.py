import asyncio
import logging
import struct
import time
import numpy as np
import cv2
import av

from aiortc import VideoStreamTrack

MAGIC_NUMBER = 0xDEADBEEF
HEADER_SIZE = 24  # 6 uint32: magic, timestamp, frame_size, width, height, pixel_format

logger = logging.getLogger(__name__)

class FrameHeader:
    def __init__(self, data: bytes):
        fields = struct.unpack("<IIIIII", data)
        self.magic = fields[0]
        self.timestamp_ms = fields[1]
        self.frame_size = fields[2]
        self.width = fields[3]
        self.height = fields[4]
        self.pixel_format = fields[5]  # 0=RGBA, 1=RGB

    @property
    def is_valid(self):
        return (
            self.magic == MAGIC_NUMBER and
            self.frame_size > 0 and
            0 < self.width <= 4000 and
            0 < self.height <= 4000
        )

class VRStreamTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, stdout_pipe):
        super().__init__()
        self.stream = stdout_pipe
        self._frame_count = 0
        self._start_time = time.time()
        self._last_log_time = self._start_time
        self._shm_frame_count = 0
        self._shm_last_log_time = self._start_time
        self._last_timestamp = None  # For duplicate filtering

    def sync_and_read_frame_header(self):
        buffer = b''
        while True:
            chunk = self.stream.read(1)
            if not chunk:
                return None
            buffer += chunk
            if len(buffer) >= 4:
                maybe_magic = struct.unpack('<I', buffer[-4:])[0]
                if maybe_magic == MAGIC_NUMBER:
                    rest = self.stream.read(HEADER_SIZE - 4)
                    if len(rest) != (HEADER_SIZE - 4):
                        return None
                    header = FrameHeader(buffer[-4:] + rest)

                    # Count only if it's a new frame (by timestamp)
                    if header.is_valid and header.timestamp_ms != self._last_timestamp:
                        self._last_timestamp = header.timestamp_ms
                        self._shm_frame_count += 1
                        now = time.time()
                        if now - self._shm_last_log_time >= 5.0:
                            elapsed = now - self._shm_last_log_time
                            fps = self._shm_frame_count / elapsed
                            logger.info(f"[VRStreamTrack] Shared memory frame FPS: {fps:.2f} ({self._shm_frame_count} frames in {elapsed:.1f}s)")
                            self._shm_frame_count = 0
                            self._shm_last_log_time = now

                    return header
                if len(buffer) > 4:
                    buffer = buffer[-3:]

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        header = self.sync_and_read_frame_header()

        if not header or not header.is_valid:
            logger.warning("Invalid or missing frame header")
            await asyncio.sleep(0.01)
            return await self.recv()

        raw_data = self.stream.read(header.frame_size)
        if not raw_data or len(raw_data) != header.frame_size:
            logger.warning(f"Expected {header.frame_size} bytes, got {len(raw_data) if raw_data else 0}")
            await asyncio.sleep(0.01)
            return await self.recv()

        arr = np.frombuffer(raw_data, np.uint8).reshape((header.height, header.width, 4))
        img = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)

        self._frame_count += 1
        now = time.time()
        if now - self._last_log_time >= 5.0:
            elapsed = now - self._last_log_time
            fps = self._frame_count / elapsed
            logger.info(f"[VRStreamTrack] Streaming FPS: {fps:.2f} ({self._frame_count} frames in {elapsed:.1f}s)")
            self._frame_count = 0
            self._last_log_time = now

        frame = av.VideoFrame.from_ndarray(img, format="bgr24")
        frame.pts = pts
        frame.time_base = time_base
        return frame
