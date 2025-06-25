import socket
import cv2
import struct
import rsa
import time
import os
from threading import Thread
from queue import Queue
import signal

try:
    from Crypto.Cipher import AES

    USE_PYCRYPTODOME = True
except ImportError:
    import pyaes

    USE_PYCRYPTODOME = False

# Global flag for shutdown, maybe I should remove the signaling
RUNNING = True


def capture_frames(cap, frame_queue):
    """Thread to capture frames and put them in a queue"""
    while RUNNING and cap.isOpened():
        ret, frame = cap.read()
        if ret:
            frame_queue.put(frame)
        else:
            break


def signal_handler(sig, frame):
    """Handle Ctrl+C for clean shutdown"""
    global RUNNING
    RUNNING = False


def server_program():
    global RUNNING
    signal.signal(signal.SIGINT, signal_handler)

    # Generate RSA keys for the first time
    """(pub_key, priv_key) = rsa.newkeys(2048)
    with open("server_private.pem", "wb") as f:s
        f.write(priv_key.save_pkcs1())
    with open("server_public.pem", "wb") as f:
        f.write(pub_key.save_pkcs1())"""
    with open("server_public copy.pem", "rb") as f:
        pub_key = rsa.PublicKey.load_pkcs1(f.read())
    with open("server_private copy.pem", "rb") as f:
        priv_key = rsa.PrivateKey.load_pkcs1(f.read())

    # UDP socket setup
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
    """
    I decided on 4MB buffer for 1080p steam. 
    TODO: make a mapping, maybe in a file or something, maybe json, which maps qualities to thes values
    """
    sock.bind(("localhost", 9999))
    sock.settimeout(20)  # initial timeout for connection just for convinience
    print("Server listening on port 9999...")

    # standard AES key reception
    try:
        data, addr = sock.recvfrom(1024)
        enc_key_len = struct.unpack("Q", data[:8])[0]
        enc_aes_key = data[8 : 8 + enc_key_len]
        iv = data[8 + enc_key_len : 8 + enc_key_len + 16]
        if not enc_aes_key or not iv:
            print("Failed to receive key or IV.")
            sock.close()
            return
        print(f"Connection from {addr}")
    except socket.timeout:
        print("Timeout receiving keys.")
        sock.close()
        return
    except Exception as e:
        print(f"Error receiving keys: {e}")
        sock.close()
        return

    aes_key = rsa.decrypt(enc_aes_key, priv_key)

    # Video capture setup
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("Failed to open webcam with V4L2, trying default backend...")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Failed to open webcam.")
            sock.close()
            return

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 60)

    # Start frame capture thread
    frame_queue = Queue(maxsize=3)  # Buffer for 1080p
    capture_thread = Thread(target=capture_frames, args=(cap, frame_queue))
    capture_thread.start()

    print("Starting video stream...")
    sequence_number = 0
    sock.settimeout(0.75)  # Reset timeout after receiving keys
    while RUNNING and cap.isOpened():
        try:
            frame = frame_queue.get(timeout=0.5)
        except:
            continue

        # Encode frame to JPEG
        encode_start = time.time()
        ret, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 20])
        encode_time = (time.time() - encode_start) * 1000
        if not ret:
            print("Failed to encode frame.")
            continue

        data = buffer.tobytes()
        # Pad for CBC
        if not USE_PYCRYPTODOME and len(data) % 16 != 0:
            pad_len = 16 - (len(data) % 16)
            data += bytes([pad_len] * pad_len)

        # Encrypt frame using the recieved AES key from the client, simple stuff
        encrypt_start = time.time()
        if USE_PYCRYPTODOME:
            aes = AES.new(aes_key, AES.MODE_GCM, nonce=iv)
            encrypted, tag = aes.encrypt_and_digest(data)
            encrypted = encrypted + tag
        else:
            aes = pyaes.AESModeOfOperationCBC(aes_key, iv=iv)
            encrypted = b""
            for i in range(0, len(data), 16):
                encrypted += aes.encrypt(data[i : i + 16])
        encrypt_time = (time.time() - encrypt_start) * 1000

        # Split into packets
        packet_size = 1400  # Larger packets for 1080p, TODO: DYNAMIC
        packets = [
            encrypted[i : i + packet_size]
            for i in range(0, len(encrypted), packet_size)
        ]
        total_packets = len(packets)
        timestamp = time.time()

        # Send packets with header: timestamp (8 bytes), seq (4 bytes), total packets (4 bytes), frame size (4 bytes)
        # This is the overhead then, I can probably shorten the seq and have wrap around behaviour
        # considering that there won't be more than a few frames in 0.03 seconds, it shouldn't inhibit much problems
        send_start = time.time()
        for i, packet in enumerate(packets):
            header = struct.pack(
                "dIII", timestamp, sequence_number, total_packets, len(encrypted)
            )
            try:
                sock.sendto(header + packet, addr)
            except Exception as e:
                print(f"Error sending packet: {e}")
                continue
        send_time = (time.time() - send_start) * 1000

        print(
            f"Frame {sequence_number} size: {len(data)} bytes, Packets: {total_packets}, Encode: {encode_time:.2f} ms, Encrypt: {encrypt_time:.2f} ms, Send: {send_time:.2f} ms"
        )
        sequence_number += 1

        if cv2.waitKey(1) & 0xFF == 13:
            """
            press q to exit I think
            will make it into a button
            """
            RUNNING = False

    print("Terminating stream.")
    try:
        sock.sendto(b"TERMINATE", addr)
    except:
        pass
    sock.close()
    cap.release()
    cv2.destroyAllWindows()
    RUNNING = False
    capture_thread.join()


if __name__ == "__main__":
    server_program()
