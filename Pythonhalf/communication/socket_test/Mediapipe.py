import cv2
import mediapipe as mp
import json
import mmap
import os
import numpy as np
import math

# Debug showing output 
showcapture = False
class HandTracker:
    def __init__(self, shm_path="../../../Shared/hands.dat"):
        # Initialize MediaPipe Holistic for complete body tracking
        self.mp_holistic = mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic(
            static_image_mode=False,
            model_complexity=1,  # 0=lite, 1=full, 2=heavy
            smooth_landmarks=True,
            enable_segmentation=False,
            smooth_segmentation=False,
            refine_face_landmarks=False,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.6
        )
        
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.running = True
        self.shm_path = shm_path
        self.shm_size = 65536

        # VR coordinate system constants
        self.SHOULDER_WIDTH_REAL = 0.45  # Average shoulder width in meters
        self.ARM_LENGTH_REAL = 0.65      # Average arm length in meters
        self.HAND_SIZE_REAL = 0.19       # Average hand span in meters
        
        # Calibration variables
        self.shoulder_width_pixels = None
        self.calibration_frames = 0
        self.calibration_complete = False

        # Create file-backed memory map
        with open(self.shm_path, "wb") as f:
            f.write(b'\x00' * self.shm_size)

        self.shm_file = open(self.shm_path, "r+b")
        self.mmap = mmap.mmap(self.shm_file.fileno(), self.shm_size)

    def write_atomic_json(self, json_bytes: bytes):
        size = len(json_bytes)
        if size + 4 >= self.shm_size:
            return  # Prevent overflow

        self.mmap.seek(4)
        self.mmap.write(json_bytes)

        # Atomic: Write size *after* writing data
        self.mmap.seek(0)
        self.mmap.write(size.to_bytes(4, byteorder='little'))

    def calibrate_shoulder_width(self, pose_landmarks):
        """Calibrate shoulder width for depth estimation"""
        if not pose_landmarks:
            return
            
        left_shoulder = pose_landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = pose_landmarks.landmark[self.mp_holistic.PoseLandmark.RIGHT_SHOULDER]
        
        # Calculate shoulder width in pixels
        shoulder_width_px = math.sqrt(
            (left_shoulder.x - right_shoulder.x)**2 + 
            (left_shoulder.y - right_shoulder.y)**2
        )
        
        if self.shoulder_width_pixels is None:
            self.shoulder_width_pixels = shoulder_width_px
        else:
            # Smooth the calibration
            self.shoulder_width_pixels = 0.9 * self.shoulder_width_pixels + 0.1 * shoulder_width_px
        
        self.calibration_frames += 1
        if self.calibration_frames > 30:  # Calibrate for 30 frames
            self.calibration_complete = True

    def calculate_vr_hand_position(self, hand_landmarks, handedness, pose_landmarks):
        """Calculate VR hand position using shoulder reference"""
        if not pose_landmarks or not self.calibration_complete:
            return self.get_basic_hand_position(hand_landmarks, handedness)
        
        # Get shoulder positions
        left_shoulder = pose_landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = pose_landmarks.landmark[self.mp_holistic.PoseLandmark.RIGHT_SHOULDER]
        
        # Calculate shoulder center
        shoulder_center = {
            'x': (left_shoulder.x + right_shoulder.x) / 2,
            'y': (left_shoulder.y + right_shoulder.y) / 2,
            'z': (left_shoulder.z + right_shoulder.z) / 2
        }
        
        # Get hand center (wrist position)
        wrist = hand_landmarks.landmark[0]  # Wrist is landmark 0
        
        # Calculate hand position relative to shoulder center
        relative_x = wrist.x - shoulder_center['x']
        relative_y = wrist.y - shoulder_center['y']
        relative_z = wrist.z - shoulder_center['z']
        
        # Estimate depth using shoulder width as reference
        depth_scale = self.SHOULDER_WIDTH_REAL / self.shoulder_width_pixels if self.shoulder_width_pixels > 0 else 1.0
        
        # Calculate hand span for additional depth estimation
        thumb_tip = hand_landmarks.landmark[4]
        pinky_tip = hand_landmarks.landmark[20]
        hand_span_px = math.sqrt(
            (thumb_tip.x - pinky_tip.x)**2 + 
            (thumb_tip.y - pinky_tip.y)**2
        )
        
        # Estimate distance based on hand size
        expected_hand_span_px = self.HAND_SIZE_REAL / depth_scale
        distance_factor = expected_hand_span_px / hand_span_px if hand_span_px > 0 else 1.0
        distance_factor = np.clip(distance_factor, 0.3, 3.0)  # Reasonable range
        
        # Convert to VR coordinates
        vr_landmarks = []
        for landmark in hand_landmarks.landmark:
            # Calculate position relative to shoulder center
            rel_x = landmark.x - shoulder_center['x']
            rel_y = landmark.y - shoulder_center['y']
            rel_z = landmark.z - shoulder_center['z']
            
            # Scale to real-world coordinates
            world_x = rel_x * depth_scale * distance_factor
            world_y = -rel_y * depth_scale * distance_factor  # Invert Y for VR
            world_z = rel_z * depth_scale * distance_factor
            
            # Adjust for hand laterality
            if handedness == 'Left':
                world_x = -abs(world_x)  # Left hand should be on left side
            else:
                world_x = abs(world_x)   # Right hand should be on right side
            
            # Position relative to VR shoulder position
            vr_x = world_x
            vr_y = world_y + 1.6  # Shoulder height in VR
            vr_z = world_z + 0.5  # Forward from body
            
            vr_landmarks.append({
                'x': vr_x,
                'y': vr_y,
                'z': vr_z
            })
        
        return vr_landmarks, distance_factor, depth_scale

    def get_basic_hand_position(self, hand_landmarks, handedness):
        """Fallback method when pose is not available"""
        landmarks = []
        for lm in hand_landmarks.landmark:
            # Basic coordinate transformation
            x = (1.0 - lm.x) if handedness == 'Right' else lm.x  # Mirror for natural interaction
            y = 1.0 - lm.y  # Invert Y
            z = lm.z * 0.5   # Basic depth
            
            landmarks.append({'x': x, 'y': y, 'z': z})
        
        return landmarks, 1.0, 1.0

    def process_frame(self, frame):
        # Flip frame horizontally for natural mirror effect
        frame = cv2.flip(frame, 1)
        
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.holistic.process(rgb_frame)
        hand_data = []

        # Calibrate shoulder width if pose is detected
        if results.pose_landmarks and not self.calibration_complete:
            self.calibrate_shoulder_width(results.pose_landmarks)

        # Process hands with pose context
        if results.left_hand_landmarks or results.right_hand_landmarks:
            
            # Process left hand
            if results.left_hand_landmarks:
                vr_landmarks, distance, depth_scale = self.calculate_vr_hand_position(
                    results.left_hand_landmarks, 'Left', results.pose_landmarks
                )
                
                hand_data.append({
                    'handedness': 'Left',
                    'landmarks': vr_landmarks,
                    'distance_factor': distance,
                    'depth_scale': depth_scale,
                    'confidence': 0.95,
                    'shoulder_calibrated': self.calibration_complete
                })

                # Draw landmarks
                self.mp_draw.draw_landmarks(
                    frame, results.left_hand_landmarks, 
                    self.mp_holistic.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )

            # Process right hand
            if results.right_hand_landmarks:
                vr_landmarks, distance, depth_scale = self.calculate_vr_hand_position(
                    results.right_hand_landmarks, 'Right', results.pose_landmarks
                )
                
                hand_data.append({
                    'handedness': 'Right',
                    'landmarks': vr_landmarks,
                    'distance_factor': distance,
                    'depth_scale': depth_scale,
                    'confidence': 0.95,
                    'shoulder_calibrated': self.calibration_complete
                })

                # Draw landmarks
                self.mp_draw.draw_landmarks(
                    frame, results.right_hand_landmarks, 
                    self.mp_holistic.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )

        # Draw pose landmarks for reference (shoulders)
        if results.pose_landmarks and showcapture:
            # Only draw shoulder landmarks
            shoulder_connections = [
                (self.mp_holistic.PoseLandmark.LEFT_SHOULDER, self.mp_holistic.PoseLandmark.RIGHT_SHOULDER)
            ]
            
            for connection in shoulder_connections:
                start_landmark = results.pose_landmarks.landmark[connection[0]]
                end_landmark = results.pose_landmarks.landmark[connection[1]]
                
                start_point = (int(start_landmark.x * frame.shape[1]), int(start_landmark.y * frame.shape[0]))
                end_point = (int(end_landmark.x * frame.shape[1]), int(end_landmark.y * frame.shape[0]))
                
                cv2.line(frame, start_point, end_point, (0, 255, 0), 2)
                cv2.circle(frame, start_point, 5, (255, 0, 0), -1)
                cv2.circle(frame, end_point, 5, (255, 0, 0), -1)

        # Display calibration status
        if showcapture:
            status_text = "Calibrated" if self.calibration_complete else f"Calibrating... {self.calibration_frames}/30"
            cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if self.calibration_complete else (0, 255, 255), 2)

        try:
            json_bytes = json.dumps(hand_data, ensure_ascii=False).encode('utf-8')
            self.write_atomic_json(json_bytes)
        except Exception as e:
            print(f"Shared memory write error: {e}")

        return frame

    def run(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: Could not open camera")
            return

        # Set camera properties for better tracking
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)

        print("Running VR Holistic Hand Tracker.")
        print("Stand with shoulders visible for calibration...")
        print("Press ESC to exit.")

        while self.running:
            success, frame = cap.read()
            if not success:
                print("Failed to grab frame")
                continue

            frame = self.process_frame(frame)
            if showcapture:
                cv2.imshow('VR Holistic Hand Tracking', frame)

            if cv2.waitKey(1) & 0xFF == 27:
                break

        cap.release()
        cv2.destroyAllWindows()
        self.cleanup()

    def stop(self):
        self.running = False

    def cleanup(self):
        self.stop()
        try:
            if self.mmap:
                self.mmap.close()
                self.mmap = None
        except Exception as e:
            print(f"Error closing mmap: {e}")

        try:
            if self.shm_file:
                self.shm_file.close()
                self.shm_file = None
        except Exception as e:
            print(f"Error closing shm_file: {e}")

        try:
            if os.path.exists(self.shm_path):
                os.remove(self.shm_path)
        except Exception as e:
            print(f"Error deleting shared memory file: {e}")


if __name__ == "__main__":
    tracker = HolisticVRHandTracker()
    tracker.run()
