import asyncio
import logging
import struct
import time
import numpy as np
import cv2
import av
from av import VideoFrame

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

    @staticmethod
    def size():
        return HEADER_SIZE

    @staticmethod
    def from_bytes(data: bytes):
        return FrameHeader(data)

class VRStreamTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, stdout_pipe, vr_debugging=False):
        super().__init__()
        self.stream = stdout_pipe
        self._frame_count = 0
        self._start_time = time.time()
        self._last_log_time = self._start_time
        self._shm_frame_count = 0
        self._shm_last_log_time = self._start_time
        self._last_timestamp = None  # For duplicate filtering
        self.vr_debugging = vr_debugging
        self._debug_window_name = "VR Debug View"
        if self.vr_debugging:
            cv2.namedWindow(self._debug_window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self._debug_window_name, 800, 600)  # Reasonable default size

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
        try:
            frame = None
            pts, time_base = await self.next_timestamp()

            # Try to read frame from shared memory
            if hasattr(self, 'frame_reader') and self.frame_reader:
                frame_data = self.frame_reader.read_frame()
                if frame_data is not None:
                    frame = frame_data
                    self._shm_frame_count += 1
                    current_time = time.time()
                    if current_time - self._shm_last_log_time >= 1.0:
                        fps = self._shm_frame_count / (current_time - self._shm_last_log_time)
                        print(f"Shared Memory FPS: {fps:.2f}")
                        self._shm_frame_count = 0
                        self._shm_last_log_time = current_time

            # If frame is not None and debugging is enabled, display it
            if frame is not None and self.vr_debugging:
                try:
                    # Convert BGR to RGB for display
                    display_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    cv2.imshow(self._debug_window_name, display_frame)
                    cv2.waitKey(1)  # Allow window to update, but don't block
                except Exception as e:
                    print(f"Error displaying debug frame: {e}")

            if frame is not None:
                # Convert to VideoFrame and return
                video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
                video_frame.pts = pts
                video_frame.time_base = time_base
                return video_frame

            # Fallback to reading from pipe if shared memory read failed
            while True:
                header_data = self.stream.read(FrameHeader.size())
                if not header_data:
                    break
                
                header = FrameHeader.from_bytes(header_data)
                
                if header.frame_size == 0:
                    continue
                
                frame_data = self.stream.read(header.frame_size)
                if not frame_data:
                    break

                # Skip duplicate frames
                if self._last_timestamp is not None and header.timestamp_ms == self._last_timestamp:
                    continue
                self._last_timestamp = header.timestamp_ms
                
                # Update frame count and calculate FPS
                self._frame_count += 1
                current_time = time.time()
                if current_time - self._last_log_time >= 1.0:
                    fps = self._frame_count / (current_time - self._last_log_time)
                    print(f"Pipe FPS: {fps:.2f}")
                    self._frame_count = 0
                    self._last_log_time = current_time

                try:
                    # Convert bytes to numpy array
                    frame = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = frame.reshape((header.height, header.width, 4))
                    frame = frame[:, :, :3]  # Convert RGBA to RGB
                    
                    # If debugging is enabled, display the frame
                    if self.vr_debugging:
                        try:
                            cv2.imshow(self._debug_window_name, frame)
                            cv2.waitKey(1)  # Allow window to update, but don't block
                        except Exception as e:
                            print(f"Error displaying debug frame: {e}")

                    # Convert to VideoFrame and return
                    video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
                    video_frame.pts = pts
                    video_frame.time_base = time_base
                    return video_frame
                except Exception as e:
                    print(f"Error processing frame: {e}")
                    continue

            # If we get here, the stream is closed
            raise ConnectionError("Stream closed")
            
        except Exception as e:
            print(f"Error in recv: {e}")
            raise

    def __del__(self):
        if self.vr_debugging:
            cv2.destroyWindow(self._debug_window_name)
