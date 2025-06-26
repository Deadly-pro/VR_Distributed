import asyncio
import base64
import json
import logging
import struct
import time
import traceback
import os
import platform
from threading import Thread, Event
from queue import Queue
import subprocess
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

# --- Toggle Options ---
USE_H264 = False      # Set False to use JPEG
USE_GPU = True       # If True, will use GPU encoder like NVIDIA's NVENC (FFmpeg needed)


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
        self.frame_queue = Queue(maxsize=3)
        self.capture_thread: Optional[Thread] = None
        self.capture_ready = Event()
        self.sequence_number = 0
        self.frame_width = 1920  # Expected VR frame dimensions
        self.frame_height = 1080
        self.fps = 60
        self.jpeg_quality = 20
        
        # VR executable path - adjust as needed
        self.vr_exe_path = os.path.join("..", "..", "test_main.py")  # Relative path from consumer location
        # Alternative: use absolute path
        # self.vr_exe_path = r"L:\combined\VR_Distributed\cpp_src\x64\Debug\VRenv(raylib).exe"

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

    def capture_frames_from_vr(self):
        '''
        Captures frames from the VR subprocess in a separate thread.
        This replaces the camera capture functionality.
        '''
        logger.info("VR capture thread started")
        
        frame_count = 0
        buffer = b''
        magic_bytes = b'\xef\xbe\xad\xde'  # 0xDEADBEEF in little-endian
        
        # Pre-allocate for performance
        expected_frame_size = 12 + (self.frame_width * self.frame_height * 4)
        
        try:
            self.capture_ready.set()
            logger.info("Starting VR frame capture...")
            
            while self.running and self.vr_process and self.vr_process.poll() is None:
                try:
                    # Read in larger chunks for efficiency
                    chunk_size = min(65536, expected_frame_size - len(buffer) if len(buffer) < expected_frame_size else 65536)
                    chunk = self.vr_process.stdout.read(chunk_size)
                    
                    if not chunk:
                        time.sleep(0.001)  # Small delay to prevent busy waiting
                        continue
                        
                    buffer += chunk
                    
                    # Process complete frames
                    while len(buffer) >= 12:
                        # Look for magic number at start of buffer
                        if not buffer.startswith(magic_bytes):
                            magic_pos = buffer.find(magic_bytes)
                            if magic_pos < 0:
                                # Keep last 11 bytes in case magic number is split
                                buffer = buffer[-11:] if len(buffer) > 11 else b''
                                break
                            buffer = buffer[magic_pos:]
                        
                        # Parse header
                        try:
                            magic, width, height = struct.unpack("<III", buffer[:12])
                            
                            if magic != 0xDEADBEEF:
                                buffer = buffer[1:]
                                continue
                            
                            # Validate dimensions
                            if width <= 0 or height <= 0 or width > 4000 or height > 4000:
                                buffer = buffer[1:]
                                continue
                            
                            image_size = width * height * 4
                            total_frame_size = 12 + image_size
                            
                            # Wait for complete frame
                            if len(buffer) < total_frame_size:
                                break
                            
                            # Extract and process frame
                            pixel_data = buffer[12:total_frame_size]
                            
                            # Convert to numpy array
                            rgba = np.frombuffer(pixel_data, dtype=np.uint8).reshape((height, width, 4))
                            rgba = np.flipud(rgba)  # Flip vertically for OpenCV
                            bgr = cv2.cvtColor(rgba[:,:,:3], cv2.COLOR_RGB2BGR)  # Drop alpha channel
                            
                            # Add frame to queue (same logic as camera capture)
                            if not self.frame_queue.full():
                                self.frame_queue.put(bgr)
                            else:
                                # Remove oldest frame and add new one
                                try:
                                    self.frame_queue.get_nowait()
                                except:
                                    pass
                                self.frame_queue.put(bgr)
                                logger.warning("Frame queue was full, replaced oldest frame")
                            
                            frame_count += 1
                            if frame_count % 30 == 0:  # Print every 30 frames
                                logger.debug(f"VR frames processed: {frame_count}")
                            
                            # Remove processed frame
                            buffer = buffer[total_frame_size:]
                            
                        except struct.error:
                            buffer = buffer[1:]
                            continue
                        except Exception as e:
                            logger.error(f"Frame processing error: {e}")
                            buffer = buffer[1:]
                            continue
                            
                except Exception as e:
                    logger.error(f"Error reading from VR process: {e}")
                    break
                    
        except Exception as e:
            logger.error(f"Error in capture_frames_from_vr: {e}")
        finally:
            logger.info(f"VR capture thread ended. Total frames: {frame_count}")

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
        This replaces the camera initialization.
        '''
        try:
            # Determine the actual executable path
            if self.vr_exe_path.endswith('.py'):
                # If it's a Python script, run it with Python
                exe_path = os.path.join(os.path.dirname(self.vr_exe_path), "cpp_src", "x64", "Debug", "VRenv(raylib).exe")
            else:
                exe_path = self.vr_exe_path
            
            logger.info(f"Launching VR process: {exe_path}")
            
            if not os.path.exists(exe_path):
                logger.error(f"VR executable not found at {exe_path}")
                return False
            
            self.vr_process = subprocess.Popen(
                [exe_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,  # Suppress stderr completely
                stdin=subprocess.PIPE,
                bufsize=0
            )
            
            logger.info(f"VR subprocess started with PID: {self.vr_process.pid}")
            
            # Test if process started correctly
            if self.vr_process.poll() is not None:
                logger.error("VR process terminated immediately")
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

    def encode_h264_with_ffmpeg(self, frame, width, height):
        command = [
            "ffmpeg", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{width}x{height}", "-i", "-",
            "-c:v", "libopenh264",  # fallback encoder
            "-f", "h264", "-"       # output to stdout
        ]

        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            out, err = process.communicate(input=frame.tobytes(), timeout=3.0)

            if process.returncode != 0:
                logger.error(f"FFmpeg failed: {err.decode()}")
                return None

            return out

        except subprocess.TimeoutExpired:
            process.kill()
            logger.error("FFmpeg encode timed out")
            return None

        except Exception as e:
            logger.error(f"Exception during FFmpeg encode: {e}")
            return None

    async def _stream_video(self):
        '''
        This method runs in a separate asyncio task to stream video frames.
        It reads frames from the VR process, encodes them as JPEG or H264, encrypts them using AES,
        and sends them to the client.
        '''
        logger.info("VR stream video started")
        try:
            while self.running and self.vr_process and self.vr_process.poll() is None:
                if self.frame_queue.empty():
                    await asyncio.sleep(0.01)
                    continue

                frame = self.frame_queue.get_nowait()
                
                # Get actual frame dimensions for encoding
                frame_height, frame_width = frame.shape[:2]
                
                if USE_H264:
                    encoded_data = self.encode_h264_with_ffmpeg(frame, frame_width, frame_height)
                    if not encoded_data:
                        continue  # Skip frame if encode failed
                else:
                    ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                    if not ret:
                        logger.warning("JPEG encoding failed")
                        continue
                    encoded_data = buffer.tobytes()

                nonce = os.urandom(12)
                cipher = AES.new(self.aes_key, AES.MODE_GCM, nonce=nonce)
                ciphertext, tag = cipher.encrypt_and_digest(encoded_data)
                timestamp = time.time()
                total_size = len(nonce) + len(ciphertext) + len(tag)
                header = struct.pack("dII", timestamp, self.sequence_number, total_size)
                payload = header + nonce + ciphertext + tag

                await self.send(bytes_data=payload)
                self.sequence_number += 1

        except Exception as e:
            logger.error(f"VR streaming error: {e}\n{traceback.format_exc()}")
        finally:
            logger.info("VR stream video ended")
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
                    '''
                    Starts the VR stream by exchanging an AES key.
                    The client sends an encrypted AES key and IV, which the server decrypts
                    using its private RSA key. The decrypted AES key is then used to encrypt
                    the video stream.
                    '''
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
                    '''
                    Just pauses the video stream. The VR process will be terminated. But the channel will remain open.
                    The client can resume the stream later.
                    '''
                    self.running = False
                    await self.send(text_data=json.dumps({'type': 'status', 'message': 'VR stream paused!'}))

                case 'resume':
                    '''
                    Resumes the paused stream. The VR process will be restarted if it was terminated.
                    '''
                    if not self.running:
                        logger.info("Resuming VR stream")
                        self.running = True
                        if not self.stream_task or self.stream_task.done():
                            if await self._initialize_vr_process():
                                self.stream_task = asyncio.create_task(self._stream_video())
                        await self.send(text_data=json.dumps({'type': 'status', 'message': 'VR stream resumed'}))

                case 'quality':
                    '''
                    Just adjusts the JPEG quality of the stream. Could probably be made dynamic based on the network conditions.
                    '''
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