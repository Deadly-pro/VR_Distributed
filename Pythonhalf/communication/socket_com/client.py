import socket
import cv2
import struct
import rsa
import os
import numpy as np
import time
from threading import Thread
from queue import Queue
import signal

try:
    from Crypto.Cipher import AES

    USE_PYCRYPTODOME = True
except ImportError:
    import pyaes

    USE_PYCRYPTODOME = False

# Global flag for clean shutdown, kinda
RUNNING = True


def display_frames(frame_queue):
    """Thread to display frames from queue"""
    while RUNNING:
        try:
            frame = frame_queue.get(timeout=0.5)
            start_time = time.time()
            cv2.imshow("Stream", frame)
            display_time = (time.time() - start_time) * 1000
            print(f"Display time: {display_time:.2f} ms")
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        except:
            continue
    cv2.destroyAllWindows()


def signal_handler(sig, frame):
    """Handle Ctrl+C for clean shutdown"""
    global RUNNING
    RUNNING = False


def client_program():
    global RUNNING
    signal.signal(
        signal.SIGINT, signal_handler
    )  # tried fixing the thread issue, didn't work this way. TODO: fix this

    # Generate AES key and IV/nonce, unique keys per every connection because why not
    aes_key = os.urandom(32)
    iv = os.urandom(16)

    # Load server public key, have to look up how this usually happens
    try:
        with open("server_public copy.pem", "rb") as f:
            pub_key = rsa.PublicKey.load_pkcs1(f.read())
    except FileNotFoundError:
        print("Error: server_public.pem not found.")
        return

    # the power of symmetric asymmetric encryption
    enc_aes_key = rsa.encrypt(aes_key, pub_key)

    # UDP socket setup
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.setsockopt(
        socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024
    )  # 4MB buffer
    client_socket.settimeout(20)  # initial timeout
    server_addr = ("localhost", 9999)

    # Send keys
    try:
        client_socket.sendto(
            struct.pack("Q", len(enc_aes_key)) + enc_aes_key + iv, server_addr
        )
    except Exception as e:
        print(f"Error sending keys: {e}")
        client_socket.close()
        return

    # Start display thread. The thread was used to both simplicity and because I wanted to see if spawning a thread still gave low latency
    frame_queue = Queue(maxsize=3)  # Buffer for 1080p
    display_thread = Thread(
        target=display_frames, args=(frame_queue,)
    )  # nice way to handle streams, thank you to the kind stackoverflow guy who blessed me with this
    display_thread.start()

    # Video reception loop, I may need to spawn a thread just for this
    packets = {}
    last_sequence = -1
    client_socket.settimeout(0.75)  # Reset timeout after sending keys
    while RUNNING:
        try:
            data, _ = client_socket.recvfrom(65535)
            if data == b"TERMINATE":
                break

            # Validate packet size (some corrupted packets had less than 20 bytes, have to look into why that is)
            if len(data) < 20:
                print(f"Packet too small ({len(data)} bytes), skipping.")
                continue

            # Parse header
            header = data[:20]
            packet_data = data[20:]
            try:
                timestamp, seq, total_packets, frame_size = struct.unpack(
                    "dIII", header
                )
            except struct.error as e:
                print(f"Error unpacking header: {e}")
                continue

            # Drop stale frames, this has to be made dynamic probably
            if time.time() - timestamp > 0.03:
                print(f"Frame {seq} too old, skipping.")
                continue

            # Store packets and the sequence in a dictionary, I'm just simulating tcp at this point without the packet recalls
            if seq not in packets:
                packets[seq] = []
            packets[seq].append(packet_data)

            # Check if frame is complete
            if len(packets[seq]) == total_packets:
                encrypted_frame = b"".join(packets[seq])
                if len(encrypted_frame) != frame_size:
                    print(
                        f"Frame {seq} incomplete (got {len(encrypted_frame)} bytes, expected {frame_size}), skipping."
                    )
                    del packets[
                        seq
                    ]  # I may need to come up with a better way to handle this.
                    continue

                # Decrypt frame
                decrypt_start = time.time()
                if USE_PYCRYPTODOME:
                    aes = AES.new(aes_key, AES.MODE_GCM, nonce=iv)
                    tag = encrypted_frame[-16:]
                    encrypted_frame = encrypted_frame[:-16]
                    try:
                        decrypted = aes.decrypt_and_verify(encrypted_frame, tag)
                    except ValueError:
                        print(f"Frame {seq} decryption failed, skipping.")
                        del packets[seq]
                        continue
                else:
                    aes = pyaes.AESModeOfOperationCBC(aes_key, iv=iv)
                    decrypted = b""
                    for i in range(0, len(encrypted_frame), 16):
                        decrypted += aes.decrypt(encrypted_frame[i : i + 16])
                    pad_len = decrypted[-1]
                    if pad_len <= 16:
                        decrypted = decrypted[:-pad_len]
                decrypt_time = (time.time() - decrypt_start) * 1000

                # Decode frame using opencv, have to look up if there exist better options
                decode_start = time.time()
                frame = cv2.imdecode(np.frombuffer(decrypted, dtype=np.uint8), 1)
                decode_time = (time.time() - decode_start) * 1000

                if frame is not None:
                    frame_queue.put(
                        frame
                    )  # insert the frame into the frame queue to be used by the spawned thread
                    latency = (time.time() - timestamp) * 1000
                    print(
                        f"Frame {seq} size: {frame_size} bytes, Packets: {total_packets}, Decrypt: {decrypt_time:.2f} ms, Decode: {decode_time:.2f} ms, Latency: {latency:.2f} ms"
                    )

                del packets[seq]
                last_sequence = seq

            # Clean up old packets
            for old_seq in list(packets.keys()):
                if old_seq < last_sequence - 1:
                    del packets[old_seq]

        except socket.timeout:
            continue
        except Exception as e:
            print(f"Error receiving packet: {e}")
            continue

    client_socket.close()
    RUNNING = False
    frame_queue.put(None)
    display_thread.join()


if __name__ == "__main__":
    client_program()
