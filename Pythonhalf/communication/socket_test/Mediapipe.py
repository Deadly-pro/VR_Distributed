import cv2
import mediapipe as mp
import json
import mmap
import os
#debug showing output 
showcapture = False
class HandTracker:
    def __init__(self, shm_path="../../../Shared/hands.dat"):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.running = True
        self.shm_path = shm_path
        self.shm_size = 65536

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

    def process_frame(self, frame):
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        hand_data = []

        if results.multi_hand_landmarks:
            for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                handedness = results.multi_handedness[idx].classification[0].label
                landmarks = [{'x': lm.x, 'y': lm.y, 'z': lm.z} for lm in hand_landmarks.landmark]

                hand_data.append({
                    'handedness': handedness,
                    'landmarks': landmarks
                })

                self.mp_draw.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

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

        print("Running hand tracker. Press ESC to exit.")

        while self.running:
            success, frame = cap.read()
            if not success:
                print("Failed to grab frame")
                continue

            frame = self.process_frame(frame)
            if showcapture:
                cv2.imshow('Hand Tracking', frame)

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
    tracker = HandTracker()
    tracker.run()