import mmap
import json
import pathlib 
import os 
class SharedMemoryHandler:
    def __init__(self, shm_path, shm_size):
        #print("test")
        base_path = pathlib.Path("__file__") / "../../../Shared"
        #print(base_path)
        self.shm_path = base_path/shm_path
        self.shm_size = shm_size
 
        # Create file-backed memory map
        with open(self.shm_path, "wb") as f:
            f.write(b'\x00' * self.shm_size)

        self.shm_file = open(self.shm_path, "r+b")
        self.mmap = mmap.mmap(self.shm_file.fileno(), self.shm_size)
    def write_data(self, data): 
        try: 
            json_data = json.dumps(data).encode('utf-8')
            if len(json_data) < self.shm_size:
                self.mmap.seek(0)
                self.mmap.write(json_data)
                self.mmap.write(b'\x00')  # Null-terminate the string
                #print("Data written to shared memory.")
        except Exception as e:
            print(f"Shared memory write error: {e}")
    def cleanup(self):
        self.mmap.close()
        self.shm_file.close()
        os.remove(self.shm_path)