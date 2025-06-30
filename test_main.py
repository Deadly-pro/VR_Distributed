import subprocess
import struct
import numpy as np
import cv2
import os
import threading
import time

def run_headless_raylib_viewer(exe_path):
    print(f"Launching: {exe_path}")
    
    if not os.path.exists(exe_path):
        print(f"ERROR: Executable not found at {exe_path}")
        return
    
    process = subprocess.Popen(
        [exe_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,  # Suppress stderr completely
        stdin=subprocess.PIPE,
        bufsize=0
    )
    print(f"Subprocess started with PID: {process.pid}")
    
    frame_count = 0
    buffer = b''
    magic_bytes = b'\xef\xbe\xad\xde'  # 0xDEADBEEF in little-endian
    
    # Pre-allocate for performance
    expected_width, expected_height = 1920, 1080
    expected_frame_size = 12 + (expected_width * expected_height * 4)
    
    try:
        print("Starting frame capture...")
        
        while True:
            # Check if process is still running
            if process.poll() is not None:
                print(f"Process terminated with return code: {process.returncode}")
                break
            
            # Read in larger chunks for efficiency
            chunk_size = min(65536, expected_frame_size - len(buffer) if len(buffer) < expected_frame_size else 65536)
            chunk = process.stdout.read(chunk_size)
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
                    
                    # Display frame
                    cv2.imshow("VR Output", bgr)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        return
                    
                    frame_count += 1
                    if frame_count % 30 == 0:  # Print every 30 frames
                        print(f"Frames processed: {frame_count}")
                    
                    # Remove processed frame
                    buffer = buffer[total_frame_size:]
                    
                except struct.error:
                    buffer = buffer[1:]
                    continue
                except Exception as e:
                    print(f"Frame processing error: {e}")
                    buffer = buffer[1:]
                    continue
    
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        print(f"Cleaning up... Total frames: {frame_count}")
        process.terminate()
        process.wait()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    exe_path = os.path.join("cpp_src", "x64", "Debug", "VRenv(raylib).exe")
    run_headless_raylib_viewer(exe_path)