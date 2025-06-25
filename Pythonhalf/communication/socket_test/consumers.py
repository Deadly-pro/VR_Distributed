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
        Initializes the consumer, sets up the camera, and prepares for video streaming.
        '''
        super().__init__(*args, **kwargs)
        self.running = False
        self.cap: Optional[cv2.VideoCapture] = None
        self.aes_key: Optional[bytes] = None
        self.iv: Optional[bytes] = None
        self.stream_task: Optional[asyncio.Task] = None
        self.frame_queue = Queue(maxsize=3)
        self.capture_thread: Optional[Thread] = None
        self.capture_ready = Event()  # Added to signal when capture is ready
        self.sequence_number = 0
        self.frame_width = 1280
        self.frame_height = 720
        self.fps = 30
        self.jpeg_quality = 20

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

    def capture_frames(self):
        '''
        Captures frames from the camera in a separate thread.
        '''
        logger.info("Capture thread started")
        self.capture_ready.set()

        while self.running and self.cap and self.cap.isOpened():
            try:
                ret, frame = self.cap.read()
                if not ret:
                    logger.error("Failed to capture frame")
                    break

                if not self.frame_queue.full():
                    self.frame_queue.put(frame)
                    # logger.debug(f"Captured frame {self.sequence_number} at {time.time():.2f} seconds")
                else:
                    # Remove oldest frame and add new one, maybe I should use a circular buffer instead?
                    try:
                        self.frame_queue.get_nowait()
                    except:
                        pass
                    self.frame_queue.put(frame)
                    logger.warning("Frame queue was full, replaced oldest frame")
            except Exception as e:
                logger.error(f"Error in capture_frames: {e}")
                break

        logger.info("Capture thread ended")

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
        This method will stop the camera stream and release the camera resources.
        '''
        await self._cleanup()

    async def _initialize_camera(self):
        '''
        Initializes the camera and sets up the video capture properties.
        Tries to open the camera using different backends based on the platform.
        '''
        try:
            backends = [
                cv2.CAP_DSHOW if platform.system() == "Windows" else cv2.CAP_V4L2,
                cv2.CAP_ANY
            ]

            for backend in backends:
                self.cap = cv2.VideoCapture(0, backend)
                if self.cap.isOpened():
                    break

            if not self.cap or not self.cap.isOpened():
                logger.error("Failed to open camera")
                return False

            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            ret, test_frame = self.cap.read()
            if not ret:
                logger.error("Failed to capture test frame")
                return False

            self.capture_ready.clear()
            self.running = True
            self.capture_thread = Thread(target=self.capture_frames, daemon=True)
            self.capture_thread.start()

            if not self.capture_ready.wait(timeout=5.0):
                logger.error("Capture thread failed to start within timeout")
                return False

            logger.info("Camera initialized")
            return True

        except Exception as e:
            logger.error(f"Camera init failed: {e}")
            return False

    def encode_h264_with_ffmpeg(self,frame, width, height):
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
        It reads frames from the camera, encodes them as JPEG or H264, encrypts them using AES,
        and sends them to the client.
        '''
        logger.info("Stream video started")
        try:
            while self.running and self.cap and self.cap.isOpened():
                if self.frame_queue.empty():
                    await asyncio.sleep(0.01)
                    continue

                frame = self.frame_queue.get_nowait()
                if USE_H264:
                    encoded_data = self.encode_h264_with_ffmpeg(frame, self.frame_width, self.frame_height)
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
            logger.error(f"Streaming error: {e}\n{traceback.format_exc()}")
        finally:
            logger.info("Stream video ended")
            await self._cleanup()

    async def _send_error(self, message):
        '''
        Convinient method to send an error message to the client.
        This method sends a JSON message with the type 'error' and the provided message.
        '''
        await self.send(text_data=json.dumps({'type': 'error', 'message': message}))

    async def _cleanup(self):
        '''
        Cleans up the resources used by the consumer.
        This method stops the camera stream, releases the camera resources, and clears the frame queue.
        '''
        logger.info("Cleaning up")
        self.running = False

        if self.stream_task and not self.stream_task.done():
            self.stream_task.cancel()
            try:
                await self.stream_task
            except asyncio.CancelledError:
                logger.info("Stream task was cancelled")

        if self.capture_thread and self.capture_thread.is_alive():
            logger.info("Waiting for capture thread to finish")
            self.capture_thread.join(timeout=2.0)
            if self.capture_thread.is_alive():
                logger.warning("Capture thread did not finish within timeout")

        if self.cap:
            self.cap.release()
            self.cap = None

        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except:
                break

        logger.info("Cleanup completed")

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
                    Starts the camera stream by exchanging an AES key.
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

                    if await self._initialize_camera():
                        self.stream_task = asyncio.create_task(self._stream_video())
                        await self.send(text_data=json.dumps({
                            'type': 'stream_ready',
                            'message': 'Video stream started'
                        }))
                    else:
                        await self._send_error("Failed to initialize camera")

                case 'pause':
                    '''
                    Just pauses the video stream. The camera will be closed. But the channel will remain open.
                    The client can resume the stream later.
                    '''
                    self.running = False
                    await self.send(text_data=json.dumps({'type': 'status', 'message': 'Stream paused!'}))

                case 'resume':
                    '''
                    Resumes the paused stream. The camera will be reopened if it was closed.
                    '''
                    if not self.running:
                        logger.info("Resuming stream")
                        self.running = True
                        if not self.stream_task or self.stream_task.done():
                            if await self._initialize_camera():
                                self.stream_task = asyncio.create_task(self._stream_video())
                        await self.send(text_data=json.dumps({'type': 'status', 'message': 'Stream resumed'}))

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
                    await self._send_error("Stream terminated by client")
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
                    #print(f"Gyroscope - α: {alpha:.2f}, β: {beta:.2f}, γ: {gamma:.2f}, t: {timestamp}")
                case _:
                    await self._send_error("Unknown message type")

        except Exception as e:
            logger.error(f"Receive error: {e}")
            await self._send_error(f"Internal error: {str(e)}")