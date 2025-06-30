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
from Crypto.Cipher import PKCS1_v1_5
from Crypto.PublicKey import RSA
import av  # For raw frame encoding
from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCIceCandidate
from aiortc.contrib.media import MediaRelay
import io
from .consumer_helper import VRStreamTrack,logger

shared_memory = sendtocpp.SharedMemoryHandler(shm_path="gyro.dat", shm_size=65536)


class WebRTCStreamingConsumer(AsyncWebsocketConsumer):
    def __init__(self, *args, **kwargs):
        '''
        WebRTC-enabled consumer for VR streaming with signaling support.
        Handles WebRTC signaling (SDP exchange, ICE candidates) and control messages.
        '''
        super().__init__(*args, **kwargs)
        self.running = False
        self.vr_process: Optional[subprocess.Popen] = None
        self.aes_key: Optional[bytes] = None
        self.iv: Optional[bytes] = None
        self.sequence_number = 0
        self.frame_width = 1920
        self.frame_height = 1080
        self.fps = 60
        self.pc: Optional[RTCPeerConnection] = None  # Add this line
        self.ice_candidate_queue = []
        self.early_ice_candidates = []
        self.stdout_buffer = io.BytesIO()  # Buffer for raw frame data
        # WebRTC signaling
        self.room_name = None
        self.peer_id = None
        self.peer_connections = {}
        
        # VR executable path - adjust as needed
        self.vr_exe_path = os.path.join("..", "..", "main.exe")
        vr_dir = os.path.abspath(os.path.join(self.vr_exe_path, ".."))
        self.shm_path = os.path.join(vr_dir, "Shared", "hands.dat")
        self.mediapipe = Mp.HandTracker(self.shm_path)
        
        # Load RSA keys for control message encryption
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

    def kind(self):
        return "video"

    async def connect(self):
        '''
        Establishes WebSocket connection and handles room joining for WebRTC signaling.
        Sends RSA public key for control message encryption.
        '''
        # Extract room name from URL route
        self.room_name = self.scope['url_route']['kwargs'].get('room_name', 'default')
        self.room_group_name = f'webrtc_vr_{self.room_name}'
        self.peer_id = self.channel_name
        
        # Join room group for WebRTC signaling
        await self.channel_layer.group_add(
            self.room_group_name,
            self.channel_name
        )
        
        await self.accept()
        
        if not self.pub_key or not self.priv_key:
            await self._send_error("RSA keys not available")
            await self.close()
            return

        # Send initialization data
        pub_key_b64 = base64.b64encode(self.pub_key.export_key()).decode()
        await self.send(text_data=json.dumps({
            'type': 'init',
            'rsa_public_key': pub_key_b64,
            'peer_id': self.peer_id,
            'room': self.room_name
        }))
        
        # Notify other peers in the room
        await self.channel_layer.group_send(
            self.room_group_name,
            {
                'type': 'peer_joined',
                'peer_id': self.peer_id,
            }
        )

    async def disconnect(self, close_code):
        '''
        Handles WebSocket disconnection and cleanup.
        '''
        # Close peer connection
        if self.pc:
            await self.pc.close()
            self.pc = None
            
        # Leave room group
        if hasattr(self, 'room_group_name'):
            await self.channel_layer.group_discard(
                self.room_group_name,
                self.channel_name
            )
            
            # Notify other peers
            await self.channel_layer.group_send(
                self.room_group_name,
                {
                    'type': 'peer_left',
                    'peer_id': self.peer_id,
                }
            )
        
        await self._cleanup()

    async def _initialize_vr_process(self):
        '''
        Initializes the VR subprocess for WebRTC streaming.
        The VR process will handle its own WebRTC peer connection.
        '''
        try:
            # Determine the actual executable path
            if self.vr_exe_path.endswith('.exe'):
                exe_path = os.path.join(os.path.dirname(self.vr_exe_path), "cpp_src", "x64", "Debug", "VRenv(raylib).exe")
            else:
                exe_path = self.vr_exe_path
            
            logger.info(f"Launching VR process for WebRTC: {exe_path}")
            
            if not os.path.exists(exe_path):
                logger.error(f"VR executable not found at {exe_path}")
                return False
            
            # Launch VR process with WebRTC mode
            self.vr_process = subprocess.Popen(
                [exe_path, "--webrtc", "--room", self.room_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdin=subprocess.PIPE,
                bufsize=1024*1024
            )
            
            logger.info(f"VR subprocess started with PID: {self.vr_process.pid}")
            
            # Test if process started correctly
            if self.vr_process.poll() is not None:
                logger.error("VR process terminated immediately")
                return False
            
            # Start MediaPipe process
            self.mediapipe_process = Thread(target=self.mediapipe.run, daemon=True)
            self.mediapipe_process.start()
            logger.info("Launching MediaPipe process...")
            
            if self.mediapipe_process.is_alive():
                logger.info("MediaPipe process started successfully")
            else:
                logger.error("MediaPipe process failed to start")
                return False
            
            self.running = True
            logger.info("VR process initialized successfully for WebRTC")
            return True

        except Exception as e:
            logger.error(f"VR process init failed: {e}")
            return False

    async def _send_error(self, message):
        '''
        Sends an error message to the client.
        '''
        await self.send(text_data=json.dumps({'type': 'error', 'message': message}))

    async def _cleanup(self):
        '''
        Cleans up VR process and MediaPipe resources.
        '''
        logger.info("Cleaning up VR resources")
        self.running = False

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
            
        if hasattr(self, 'mediapipe_process') and self.mediapipe_process and self.mediapipe_process.is_alive():
            logger.info("Terminating MediaPipe process")
            self.mediapipe.cleanup()
            self.mediapipe_process.join(timeout=2.0)
            if self.mediapipe_process.is_alive():
                logger.warning("MediaPipe process did not finish within timeout")
            self.mediapipe_process = None

        logger.info("VR cleanup completed")

    def decrypt_message(self, encrypted_bytes: bytes) -> Optional[str]:
        '''
        Decrypts AES-GCM encrypted control messages sent by the client.
        Format: [12 bytes nonce][ciphertext + tag]
        Note: Only used for control messages, not video frames (WebRTC handles video).
        '''
        from Crypto.Cipher import AES
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

    def _parse_ice_candidate(self, candidate_str, sdp_mid, sdp_mline_index):
        """Parse ICE candidate string and create RTCIceCandidate object for aiortc"""
        try:
            # Remove "candidate:" prefix if present
            if candidate_str.startswith("candidate:"):
                candidate_str = candidate_str[10:]
            
            parts = candidate_str.split()
            if len(parts) < 6:
                raise ValueError(f"Invalid candidate format: {candidate_str}")
            
            foundation = parts[0]
            component = int(parts[1])
            protocol = parts[2].lower()
            priority = int(parts[3])
            ip = parts[4]
            port = int(parts[5])
            
            # Find type (typ host/srflx/relay/etc)
            typ = "host"  # default
            for i, part in enumerate(parts):
                if part == "typ" and i + 1 < len(parts):
                    typ = parts[i + 1]
                    break
            
            return RTCIceCandidate(
                component=component,
                foundation=foundation,
                ip=ip,
                port=port,
                priority=priority,
                protocol=protocol,
                type=typ,
                sdpMid=sdp_mid,
                sdpMLineIndex=sdp_mline_index
            )
        except Exception as e:
            logger.error(f"Failed to parse ICE candidate: {e}")
            raise

    async def receive(self, text_data=None, bytes_data=None):
        '''
        Handles WebRTC signaling messages and control messages.
        Supports SDP exchange, ICE candidates, and VR control commands.
        '''
        try:
            data = None

            logger.debug(f"Received WebSocket message: {text_data if text_data else bytes_data}")

            if text_data:
                try:
                    # First try plaintext 
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

            logger.debug(f"Parsed data: {data}")

            if not data:
                await self._send_error("No message data received")
                return

            msg_type = data.get('type')
            if not msg_type:
                await self._send_error("Missing or invalid 'type' field in message")
                logger.error(f"Missing or invalid 'type' in message: {data}")
                return

            match msg_type:
                # Key exchange for control messages
                case 'aes_key_exchange':
                    await self._handle_key_exchange(data)
                
                # WebRTC signaling messages
                case 'webrtc_offer':
                    logger.info("Received WebRTC offer")
                    await self._handle_webrtc_offer(data)
                
                case 'webrtc_answer':
                    await self._handle_webrtc_answer(data)
                
                case 'webrtc_ice_candidate':
                    await self._handle_ice_candidate(data)
                
                # VR control messages
                case 'start_vr':
                    if await self._initialize_vr_process():
                        await self.send(text_data=json.dumps({
                            'type': 'vr_ready',
                            'message': 'VR process started and ready for WebRTC'
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
                        if not self.vr_process or self.vr_process.poll() is not None:
                            if await self._initialize_vr_process():
                                await self.send(text_data=json.dumps({'type': 'status', 'message': 'VR stream resumed'}))
                        else:
                            await self.send(text_data=json.dumps({'type': 'status', 'message': 'VR stream resumed'}))

                case 'quality':
                    value = data.get('value')
                    if isinstance(value, int) and 1 <= value <= 100:
                        # Forward quality setting to VR process if needed
                        await self.send(text_data=json.dumps({'type': 'status', 'message': f'Quality set to {value}'}))
                    else:
                        await self._send_error("Invalid quality value")

                case 'terminate':
                    self.running = False
                    await self._send_error("VR stream terminated by client")
                    await self.close()

                case 'gyro':
                    # Handle gyroscope data
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
                case 'ping':
                    logger.info("Received ping from client")
                    await self.send(text_data=json.dumps({'type': 'pong'}))

                case _:
                    await self._send_error("Unknown message type")

        except Exception as e:
            logger.error(f"Receive error: {e}")
            await self._send_error(f"Internal error: {str(e)}")

    async def _handle_key_exchange(self, data):
        '''
        Handles AES key exchange for control messages.
        '''
        try:
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

            await self.send(text_data=json.dumps({
                'type': 'key_exchange_complete',
                'message': 'AES key established for control messages'
            }))

        except Exception as e:
            logger.error(f"Key exchange error: {e}")
            await self._send_error("Key exchange failed")

    async def _handle_webrtc_offer(self, data):
        '''
        Handles WebRTC offer and sets up aiortc PeerConnection for VR streaming.
        '''
        offer = data.get('offer')
        if not offer:
            await self._send_error("No offer provided")
            return

        logger.info(f"📩 Received WebRTC offer: {offer}")
        sdp = offer.get("sdp")
        sdp_type = offer.get("type")
        if not sdp or not sdp_type:
            await self._send_error("Malformed WebRTC offer: missing sdp or type")
            logger.error(f"❌ Malformed offer: {offer}")
            return
        
        configuration = RTCConfiguration(
            iceServers=[
                RTCIceServer(
                    urls=["stun:stun.l.google.com:19302"]
                )
            ]
        )

        self.pc = RTCPeerConnection(configuration=configuration)
        # Add the track BEFORE setting remote description
        # track = DummyTrack()
        track = VRStreamTrack(self.vr_process.stdout)
        self.pc.addTrack(track)
        logger.info("DummyTrack added to PeerConnection before setRemoteDescription")

        @self.pc.on("iceconnectionstatechange")
        async def on_ice_state_change():
            logger.info(f"ICE state: {self.pc.iceConnectionState}")
            if self.pc.iceConnectionState == "failed":
                await self.pc.close()

        @self.pc.on("icecandidate")
        async def on_icecandidate(ev):
            if ev.candidate:
                logger.info(f"👾 ICE candidate generated: {ev.candidate}")
                await self.send(text_data=json.dumps({
                    "type": "webrtc_ice_candidate",
                    "candidate": {
                        "candidate": ev.candidate.candidate,
                        "sdpMid": ev.candidate.sdpMid,
                        "sdpMLineIndex": ev.candidate.sdpMLineIndex,
                    }
                }))

        try:
            await self.pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=sdp_type))
            # Flush queued ICE candidates
            for candidate_str, sdp_mid, sdp_mline_index in self.ice_candidate_queue:
                try:
                    ice_candidate = self._parse_ice_candidate(candidate_str, sdp_mid, sdp_mline_index)
                    await self.pc.addIceCandidate(ice_candidate)
                    print("[ICE] Replayed queued candidate")
                except Exception as e:
                    print(f"[ICE] Failed to add queued candidate: {e}")
            self.ice_candidate_queue.clear()
            # Flush early ICE candidates
            for cand in self.early_ice_candidates:
                try:
                    candidate_str = cand["candidate"]
                    sdp_mid = cand.get("sdpMid")
                    sdp_mline_index = cand.get("sdpMLineIndex")
                    ice_candidate = self._parse_ice_candidate(candidate_str, sdp_mid, sdp_mline_index)
                    await self.pc.addIceCandidate(ice_candidate)
                    print("[ICE] Added early buffered ICE candidate")
                except Exception as e:
                    print(f"[ICE] Failed to add early buffered ICE candidate: {e}")
            self.early_ice_candidates.clear()
        except Exception as e:
            logger.error(f"Failed to set remote description: {e}")
            await self._send_error("Failed to set remote description")
            return

        answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(answer)
        logger.info(f"📩 Created WebRTC answer: {answer.sdp}")
        logger.info("✅ Sent WebRTC answer to client")

        await self.send(text_data=json.dumps({
            "type": "answer",
            "answer": {
                "type": self.pc.localDescription.type,
                "sdp": self.pc.localDescription.sdp,
            },
            "from": self.peer_id  # if applicable
        }))

    async def _handle_webrtc_answer(self, data):
        '''
        Handles WebRTC answer (for cases where this peer initiated the connection).
        '''
        answer = data.get('answer')
        if not answer or not self.pc:
            await self._send_error("No answer provided or no peer connection")
            return

        try:
            await self.pc.setRemoteDescription(
                RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
            )
            logger.info("✅ WebRTC answer processed successfully")
        except Exception as e:
            logger.error(f"Failed to process WebRTC answer: {e}")
            await self._send_error("Failed to process WebRTC answer")

    async def _handle_ice_candidate(self, data):
        '''
        Handles ICE candidate and adds it to aiortc PeerConnection.
        '''
        cand = data.get("candidate")
        if not cand or not cand.get("candidate"):
            logger.warning("Received empty or invalid ICE candidate, skipping")
            return
        if self.pc is None:
            logger.warning("Received ICE candidate but no peer connection exists — buffering")
            self.early_ice_candidates.append(cand)
            return
        candidate_str = cand["candidate"]
        sdp_mid = cand.get("sdpMid")
        sdp_mline_index = cand.get("sdpMLineIndex")
        if not self.pc.remoteDescription:
            self.ice_candidate_queue.append((candidate_str, sdp_mid, sdp_mline_index))
            print("[ICE] Queued ICE candidate (remoteDescription not ready)")
            return
        try:
            ice_candidate = self._parse_ice_candidate(candidate_str, sdp_mid, sdp_mline_index)
            await self.pc.addIceCandidate(ice_candidate)
            logger.info(f"Added ICE candidate: {candidate_str}")
        except Exception as e:
            logger.error(f"Failed to add ICE candidate: {e}")
            logger.error(f"Candidate data: {cand}")
            traceback.print_exc()
            await self._send_error(f"Failed to add ICE candidate: {str(e)}")

    # Channel layer message handlers
    async def webrtc_offer_message(self, event):
        '''
        Sends WebRTC offer to client.
        '''
        await self.send(text_data=json.dumps({
            'type': 'webrtc_offer',
            'offer': event['offer'],
            'from': event['from_peer'],
        }))

    async def webrtc_answer_message(self, event):
        '''
        Sends WebRTC answer to client.
        '''
        await self.send(text_data=json.dumps({
            'type': 'webrtc_answer',
            'answer': event['answer'],
            'from': event['from_peer'],
        }))

    async def webrtc_ice_message(self, event):
        '''
        Sends ICE candidate to client.
        '''
        await self.send(text_data=json.dumps({
            'type': 'webrtc_ice_candidate',
            'candidate': event['candidate'],
            'from': event['from_peer'],
        }))

    async def peer_joined(self, event):
        '''
        Notifies client when a peer joins the room.
        '''
        if event['peer_id'] != self.peer_id:
            await self.send(text_data=json.dumps({
                'type': 'peer_joined',
                'peer_id': event['peer_id'],
            }))

    async def peer_left(self, event):
        '''
        Notifies client when a peer leaves the room.
        '''
        if event['peer_id'] != self.peer_id:
            await self.send(text_data=json.dumps({
                'type': 'peer_left',
                'peer_id': event['peer_id'],
            }))

# Add a placeholder for read_jpeg_frame in SharedMemoryHandler if not present
if not hasattr(sendtocpp.SharedMemoryHandler, 'read_jpeg_frame'):
    def read_jpeg_frame(self):
        # Placeholder: implement actual shared memory JPEG frame reading
        return None
    sendtocpp.SharedMemoryHandler.read_jpeg_frame = read_jpeg_frame