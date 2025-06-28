import asyncio
import base64
import json
import logging
import struct
import time
import traceback
import os
import platform
import sys
from threading import Thread, Event
from queue import Queue
import subprocess
from . import Mediapipe as Mp
import shlex
import cv2
import numpy as np
from channels.generic.websocket import AsyncWebsocketConsumer
from typing import Optional
from . import sendtocpp
from Crypto.Cipher import AES, PKCS1_v1_5
from Crypto.PublicKey import RSA

shared_memory = sendtocpp.SharedMemoryHandler(shm_path="gyro.dat", shm_size=65536)
logger = logging.getLogger(__name__)

# Frame data structure matching C++
class FrameHeader:
    def __init__(self, data: bytes):
        fields = struct.unpack("<IIIIII", data)
        self.magic = fields[0]
        self.timestamp_ms = fields[1]
        self.frame_size = fields[2]
        self.width = fields[3]
        self.height = fields[4]
        self.quality = fields[5]
    
    @property
    def is_valid(self):
        return (self.magic == 0xDEADBEEF and 
                self.frame_size > 0 and 
                self.width > 0 and self.height > 0 and
                self.width <= 4000 and self.height <= 4000)

MAGIC_NUMBER = 0xDEADBEEF
HEADER_SIZE = 24

def sync_and_read_frame_header(stream) -> Optional['FrameHeader']:
    buffer = b''
    while True:
        chunk = stream.read(1)
        if not chunk:
            return None
        buffer += chunk
        if len(buffer) >= 4:
            maybe_magic = struct.unpack('<I', buffer[-4:])[0]
            if maybe_magic == MAGIC_NUMBER:
                rest = stream.read(HEADER_SIZE - 4)
                if len(rest) != (HEADER_SIZE - 4):
                    return None
                return FrameHeader(buffer[-4:] + rest)
            if len(buffer) > 4:
                buffer = buffer[-3:]

class StreamingConsumer(AsyncWebsocketConsumer):
    def __init__(self, *args, **kwargs):
        '''
        Runs when the consumer is initialized.
        Initializes the consumer, sets up the VR subprocess, and prepares for video streaming.
        '''
        super().__init__(*args, **kwargs)
        self.running = False
        self.vr_process: Optional[subprocess.Popen] = None
        self.aes_key: Optional[bytes] = None
        self.iv: Optional[bytes] = None
        self.stream_task: Optional[asyncio.Task] = None
        self.frame_queue = Queue(maxsize=2)  # Small queue for low latency
        self.capture_thread: Optional[Thread] = None
        self.capture_ready = Event()
        self.sequence_number = 0
        self.frame_width = 1920  # Expected VR frame dimensions
        self.frame_height = 1080
        self.fps = 60
        
        # VR executable path - adjust as needed
        self.vr_exe_path = os.path.join("..", "..", "main.exe")
        vr_dir = os.path.abspath(os.path.join(self.vr_exe_path, ".."))
        self.shm_path = os.path.join(vr_dir, "Shared", "hands.dat")
        self.mediapipe=Mp.HandTracker(self.shm_path)
        # Alternative: use absolute path
        # self.vr_exe_path = r"L:\combined\VR_Distributed\cpp_src\x64\Debug\VRenv(raylib).exe"
        #self.shm_path=r"L:\VR_Distributed\Shared\hands.dat"
        try:
            with open("server_public.pem", "rb") as f:
                self.pub_key = RSA.import_key(f.read())
            with open("server_private.pem", "rb") as f:
                self.priv_key = RSA.import_key(f.read())
            logger.info("RSA keys loaded successfully")
        except Exception as e:
            logger.error(f"Failed to load RSA keys: {e}")
            self.pub_key = None
            self.priv_key = None

    def read_exact(self, stream, size: int) -> Optional[bytes]:
        """Read exactly 'size' bytes from stream with timeout"""
        data = b''
        timeout_count = 0
        max_timeout = 100  # 100ms total timeout
        
        while len(data) < size:
            try:
                chunk = stream.read(size - len(data))
                if not chunk:
                    timeout_count += 1
                    if timeout_count > max_timeout:
                        return None
                    time.sleep(0.001)  # 1ms wait
                    continue
                    
                data += chunk
                timeout_count = 0  # Reset timeout counter
                
            except Exception as e:
                logger.error(f"Read error: {e}")
                return None
                
        return data

    def capture_frames_from_vr(self):
        '''
        Captures JPEG-encoded frames from the VR subprocess in a separate thread.
        Much more efficient than the previous raw frame approach.
        '''
        logger.info("VR JPEG capture thread started")
        
        frame_count = 0
        bytes_received = 0
        start_time = time.time()
        last_stat_time = start_time
        
        try:
            self.capture_ready.set()
            logger.info("Starting VR JPEG frame capture...")
            while self.running and self.vr_process and self.vr_process.poll() is None:
                try:
                    # Use sync-and-read to find the next valid frame header
                    header = sync_and_read_frame_header(self.vr_process.stdout)
                    if not header or not header.is_valid:
                        continue
                    # Read JPEG data
                    jpeg_data = self.read_exact(self.vr_process.stdout, header.frame_size)
                    if not jpeg_data:
                        logger.warning("Failed to read JPEG data")
                        continue
                    bytes_received += HEADER_SIZE + len(jpeg_data)
                    frame_info = {
                        'jpeg_data': jpeg_data,
                        'width': header.width,
                        'height': header.height,
                        'quality': header.quality,
                        'timestamp_ms': header.timestamp_ms
                    }
                    try:
                        self.frame_queue.put_nowait(frame_info)
                    except:
                        try:
                            self.frame_queue.get_nowait()
                            self.frame_queue.put_nowait(frame_info)
                        except:
                            pass
                    frame_count += 1
                    current_time = time.time()
                    if current_time - last_stat_time >= 5.0:
                        elapsed = current_time - start_time
                        fps = frame_count / elapsed if elapsed > 0 else 0
                        mbps = (bytes_received * 8 / (1024 * 1024)) / elapsed if elapsed > 0 else 0
                        logger.info(f"VR Stats - Frames: {frame_count}, FPS: {fps:.1f}, "
                                    f"Data rate: {mbps:.2f} Mbps, Queue size: {self.frame_queue.qsize()}")
                        last_stat_time = current_time
                except Exception as e:
                    logger.error(f"Frame capture error: {e}")
                    break
        except Exception as e:
            logger.error(f"Fatal error in capture_frames_from_vr: {e}")
        finally:
            logger.info(f"VR JPEG capture thread ended. Total frames: {frame_count}")

    async def connect(self):
        '''
        Runs when the WebSocket connection is established.
        Sends the RSA public key to the client for AES key exchange.
        Then waits for the client to send the AES key.
        If the keys are not available, it sends an error message and closes the connection.
        '''
        await self.accept()
        if not self.pub_key or not self.priv_key:
            await self._send_error("RSA keys not available")
            await self.close()
            return

        pub_key_b64 = base64.b64encode(self.pub_key.export_key()).decode()
        await self.send(text_data=json.dumps({
            'type': 'rsa_public_key',
            'key': pub_key_b64
        }))

    async def disconnect(self, close_code):
        '''
        Just runs the custom cleanup method when the WebSocket connection is closed.
        This method will stop the VR stream and release the VR process resources.
        '''
        await self._cleanup()

    async def _initialize_vr_process(self):
        '''
        Initializes the VR subprocess and sets up the frame capture.
        Now expects JPEG-encoded frames instead of raw frames.
        '''
        try:
            # Determine the actual executable path
            if self.vr_exe_path.endswith('.exe'):
                # If it's a native script run directly
                exe_path = os.path.join(os.path.dirname(self.vr_exe_path), "cpp_src", "x64", "Debug", "VRenv(raylib).exe")
            else:
                exe_path = self.vr_exe_path
            
            logger.info(f"Launching VR process: {exe_path}")
            
            if not os.path.exists(exe_path):
                logger.error(f"VR executable not found at {exe_path}")
                return False
            # Use larger buffers and disable buffering
            self.vr_process = subprocess.Popen(
                [exe_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.PIPE,
                bufsize=1024*1024  # 1MB buffer
            )
            logger.info(f"VR subprocess started with PID: {self.vr_process.pid}")
            #Test if process started correctly
            if self.vr_process.poll() is not None:
                logger.error("VR process terminated immediately")
                return False
            # Mediapipe process seems to only work with proper perms in threads so no subprocess.Popen used :(
            self.mediapipe_process = Thread(target=self.mediapipe.run, daemon=True)
            self.mediapipe_process.start()
            logger.info(f"Launching Mediapipe process..")
            if self.mediapipe_process.is_alive():
                logger.info("Mediapipe process started successfully")
            else:
                logger.error("Mediapipe process failed to start")
                return False
            self.capture_ready.clear()
            self.running = True
            self.capture_thread = Thread(target=self.capture_frames_from_vr, daemon=True)
            self.capture_thread.start()

            if not self.capture_ready.wait(timeout=10.0):  # Longer timeout for VR startup
                logger.error("VR capture thread failed to start within timeout")
                return False

            logger.info("VR process initialized successfully")
            return True

        except Exception as e:
            logger.error(f"VR process init failed: {e}")
            return False

    async def _stream_video(self):
        '''
        This method runs in a separate asyncio task to stream video frames.
        Now works with pre-encoded JPEG frames from C++, eliminating the encoding bottleneck.
        '''
        logger.info("VR JPEG stream started")
        frames_sent = 0
        bytes_sent = 0
        start_time = time.time()
        last_stat_time = start_time
        
        try:
            while self.running and self.vr_process and self.vr_process.poll() is None:
                if self.frame_queue.empty():
                    await asyncio.sleep(0.001)  # Very short sleep
                    continue

                frame_info = self.frame_queue.get_nowait()
                jpeg_data = frame_info['jpeg_data']
                
                # Encrypt the JPEG data directly (no re-encoding needed!)
                nonce = os.urandom(12)
                cipher = AES.new(self.aes_key, AES.MODE_GCM, nonce=nonce)
                ciphertext, tag = cipher.encrypt_and_digest(jpeg_data)
                
                # Create payload with frame metadata
                timestamp = time.time()
                total_size = len(nonce) + len(ciphertext) + len(tag)
                
                # FIXED: Use the correct 16-byte header format that matches JavaScript expectations
                header = struct.pack("dII", 
                    timestamp,                    # 8 bytes - double: timestamp
                    self.sequence_number,         # 4 bytes - uint32: sequence
                    total_size,                   # 4 bytes - uint32: encrypted data size
                )
                # Total header size: 16 bytes (matches JavaScript headerSize = 16)
                
                payload = header + nonce + ciphertext + tag
                
                await self.send(bytes_data=payload)
                
                self.sequence_number += 1
                frames_sent += 1
                bytes_sent += len(payload)
                
                # Log statistics every 5 seconds
                current_time = time.time()
                if current_time - last_stat_time >= 5.0:
                    elapsed = current_time - start_time
                    fps = frames_sent / elapsed if elapsed > 0 else 0
                    mbps = (bytes_sent * 8 / (1024 * 1024)) / elapsed if elapsed > 0 else 0
                    
                    logger.info(f"Stream Stats - Sent: {frames_sent}, FPS: {fps:.1f}, "
                            f"Bandwidth: {mbps:.2f} Mbps, Avg frame size: {bytes_sent/frames_sent if frames_sent > 0 else 0:.0f} bytes")
                    
                    # Optional: Log frame info for debugging
                    logger.debug(f"Frame info - Width: {frame_info['width']}, Height: {frame_info['height']}, "
                            f"Quality: {frame_info['quality']}, VR timestamp: {frame_info['timestamp_ms']}")
                    
                    last_stat_time = current_time

        except Exception as e:
            logger.error(f"VR streaming error: {e}\n{traceback.format_exc()}")
        finally:
            logger.info(f"VR JPEG stream ended. Frames sent: {frames_sent}")
            await self._cleanup()


    async def _send_error(self, message):
        '''
        Convenient method to send an error message to the client.
        This method sends a JSON message with the type 'error' and the provided message.
        '''
        await self.send(text_data=json.dumps({'type': 'error', 'message': message}))

    async def _cleanup(self):
        '''
        Cleans up the resources used by the consumer.
        This method stops the VR stream, terminates the VR process, and clears the frame queue.
        '''
        logger.info("Cleaning up VR resources")
        self.running = False

        if self.stream_task and not self.stream_task.done():
            self.stream_task.cancel()
            try:
                await self.stream_task
            except asyncio.CancelledError:
                logger.info("VR stream task was cancelled")

                if self.capture_thread and self.capture_thread.is_alive():
                    logger.info("Waiting for VR capture thread to finish")
                    self.capture_thread.join(timeout=2.0)
            if self.capture_thread.is_alive():
                logger.warning("VR capture thread did not finish within timeout")

        if self.vr_process:
            logger.info("Terminating VR process")
            self.vr_process.terminate()
            try:
                self.vr_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                logger.warning("VR process did not terminate gracefully, killing it")
                self.vr_process.kill()
                self.vr_process.wait()
            self.vr_process = None
        if self.mediapipe_process and self.mediapipe_process.is_alive():
            logger.info("Terminating Mediapipe process")
            self.mediapipe.cleanup()
            self.mediapipe_process.join(timeout=2.0)
            if self.mediapipe_process.is_alive():
                logger.warning("Mediapipe process did not finish within timeout")
            self.mediapipe_process = None
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except:
                break

        logger.info("VR cleanup completed")

    def decrypt_message(self, encrypted_bytes: bytes) -> Optional[str]:
        '''
        Decrypts an AES-GCM encrypted message sent by the client.
        Format: [12 bytes nonce][ciphertext + tag]
        '''
        try:
            if not self.aes_key:
                logger.error("AES key not set")
                return None

            nonce = encrypted_bytes[:12]
            ciphertext_and_tag = encrypted_bytes[12:]

            cipher = AES.new(self.aes_key, AES.MODE_GCM, nonce=nonce)
            decrypted_data = cipher.decrypt_and_verify(
                ciphertext_and_tag[:-16],  # ciphertext
                ciphertext_and_tag[-16:]  # tag
            )
            return decrypted_data.decode('utf-8')
        except Exception as e:
            logger.error(f"Decryption failed: {e}")
            return None

    async def receive(self, text_data=None, bytes_data=None):
        '''
        Handles messages received from the client.
        Supports key exchange, quality control, pausing/resuming, gyroscope data, etc.
        '''
        try:
            data = None

            if text_data:
                try:
                    # First try plaintext JSON
                    data = json.loads(text_data)
                except json.JSONDecodeError:
                    # If that fails, treat it as encrypted Base64
                    logger.info("Attempting to decrypt Base64-encoded text_data")
                    encrypted_bytes = base64.b64decode(text_data)
                    decrypted = self.decrypt_message(encrypted_bytes)
                    if not decrypted:
                        await self._send_error("Failed to decrypt text message")
                        return
                    data = json.loads(decrypted)

            elif bytes_data:
                decrypted = self.decrypt_message(bytes_data)
                if not decrypted:
                    await self._send_error("Failed to decrypt binary message")
                    return
                data = json.loads(decrypted)

            if not data:
                await self._send_error("No message data received")
                return

            msg_type = data.get('type')

            match msg_type:
                case 'aes_key_exchange':
                    enc_key = base64.b64decode(data['encrypted_key'])
                    iv = base64.b64decode(data['iv'])
                    cipher = PKCS1_v1_5.new(self.priv_key)
                    decrypted_key_b64 = cipher.decrypt(enc_key, None)

                    if decrypted_key_b64 is None:
                        await self._send_error("AES decryption failed")
                        return

                    try:
                        decrypted_key = base64.b64decode(decrypted_key_b64)
                    except Exception as e:
                        logger.error("Base64 decode error on decrypted key: %s", e)
                        await self._send_error("Invalid decrypted AES key format")
                        return

                    if len(decrypted_key) > 32:
                        self.aes_key = decrypted_key[:32]
                    elif len(decrypted_key) in [16, 24, 32]:
                        self.aes_key = decrypted_key
                    else:
                        self.aes_key = decrypted_key.ljust(32, b'\x00')

                    self.iv = iv

                    if await self._initialize_vr_process():
                        self.stream_task = asyncio.create_task(self._stream_video())
                        await self.send(text_data=json.dumps({
                            'type': 'stream_ready',
                            'message': 'VR video stream started'
                        }))
                    else:
                        await self._send_error("Failed to initialize VR process")

                case 'pause':
                    self.running = False
                    await self.send(text_data=json.dumps({'type': 'status', 'message': 'VR stream paused'}))

                case 'resume':
                    if not self.running:
                        logger.info("Resuming VR stream")
                        self.running = True
                        if not self.stream_task or self.stream_task.done():
                            if await self._initialize_vr_process():
                                self.stream_task = asyncio.create_task(self._stream_video())
                        await self.send(text_data=json.dumps({'type': 'status', 'message': 'VR stream resumed'}))

                case 'quality':
                    value = data.get('value')
                    if isinstance(value, int) and 1 <= value <= 100:
                        self.jpeg_quality = value
                        await self.send(text_data=json.dumps({'type': 'status', 'message': f'JPEG quality set to {value}'}))
                    else:
                        await self._send_error("Invalid quality value")

                case 'terminate':
                    self.running = False
                    await self._send_error("VR stream terminated by client")
                    await self.close()

                case 'gyro':
                    alpha = data.get('alpha')
                    beta = data.get('beta')
                    gamma = data.get('gamma')
                    timestamp = data.get('timestamp')
                    shared_memory.write_data({
                        'alpha': alpha,
                        'beta': beta,
                        'gamma': gamma,
                        'timestamp': timestamp
                    })
                    logger.info(f"Gyroscope - α: {alpha:.2f}, β: {beta:.2f}, γ: {gamma:.2f}, t: {timestamp}")

                case _:
                    await self._send_error("Unknown message type")

        except Exception as e:
            logger.error(f"Receive error: {e}")
            await self._send_error(f"Internal error: {str(e)}")
