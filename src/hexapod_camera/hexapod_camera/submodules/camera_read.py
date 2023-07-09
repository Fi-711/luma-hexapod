#!/usr/bin/env python3

import cv2
import numpy as np
import sysv_ipc


class CameraRead():
    """
    Class that reads the output of the shared memory
    """
    def __init__(self, key=0xDEADBEEF, WIDTH=640, HEIGHT=480):
        # Key code for shared memory
        self.key = key
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT

        # Define the size of the shared memory buffer
        self.SHM_SIZE = WIDTH*HEIGHT*3

        # Attach to the shared memory object created by the first script
        self.shm = sysv_ipc.SharedMemory(key=self.key, flags=sysv_ipc.IPC_CREAT, mode=0o666, size=self.SHM_SIZE)

        # boolean so only one window ever used
        self.showing_empty_buffer_window = False


    def shm_exists(self):
        """
        Check if memory is being shared
        """
        try:
            self.shm = sysv_ipc.SharedMemory(self.key)
            self.shm.detach()
            self.shm = sysv_ipc.SharedMemory(key=self.key, flags=sysv_ipc.IPC_CREAT, mode=0o666, size=self.SHM_SIZE)
            return True
        except sysv_ipc.ExistentialError:
            return False
        

    def get_camera_feed(self):
        """
        Reads camera feed from the shared memory.
        """
        # Loop indefinitely
        while True:
            # Read the shared memory buffer
            buffer = self.shm.read(self.SHM_SIZE)

            # Convert the buffer to a NumPy array
            frame = np.frombuffer(buffer, dtype=np.uint8)

            # Reshape the NumPy array into a 2D grayscale image
            frame = frame.reshape((self.HEIGHT, self.WIDTH, 3))

            # if memory is being shared
            if self.shm_exists():
                # The buffer contains non-zero data, so it's probably a valid image
                # Display the image
                cv2.namedWindow("Shared Memory Feed", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Shared Memory Feed", self.WIDTH, self.HEIGHT)
                cv2.imshow("Shared Memory Feed", frame)
                
                # Exit the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # destroy error window if it exists
                if self.showing_empty_buffer_window:
                    cv2.destroyWindow("Empty Buffer! Check Video Capture")
                    self.showing_empty_buffer_window = False
            
            else:
            # The buffer is empty, so display a warning message
                frame = np.zeros((self.HEIGHT, self.WIDTH, 3), dtype=np.uint8)
                frame = frame.reshape((self.HEIGHT, self.WIDTH, 3))
                cv2.namedWindow("Empty Buffer! Check Video Capture", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Empty Buffer! Check Video Capture", self.WIDTH, self.HEIGHT)
                cv2.putText(img=frame, text="Camera Feed Disconnected...", org=(50, 240), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2)    # opencv uses BGR colour format
                cv2.imshow("Empty Buffer! Check Video Capture", frame)
                
                # Exit the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # destroy image window if it exists because it means it has frozen if code has reached here
                if not self.showing_empty_buffer_window:
                    cv2.destroyWindow("Shared Memory Feed")
                    self.showing_empty_buffer_window = True
                    
                    # detach and reatach - 'refreshing' input
                    self.shm_exists()

        # Destroy the window and detach from the shared memory object
        cv2.destroyAllWindows()
        self.shm.detach()

    
    def get_camera_frame(self):
        """
        Returns the latest camera frame from the shared memory.
        """
        # if memory is being shared
        if self.shm_exists():
        # The buffer contains non-zero data, so it's probably a valid image
            # Read the shared memory buffer
            buffer = self.shm.read(self.SHM_SIZE)

            # Convert the buffer to a NumPy array
            frame = np.frombuffer(buffer, dtype=np.uint8)

            # Reshape the NumPy array into a 2D grayscale image
            frame = frame.reshape((self.HEIGHT, self.WIDTH, 3))
        
        else:
        # The buffer is empty, so return a black frame
            frame = np.zeros((self.HEIGHT, self.WIDTH, 3), dtype=np.uint8)
            frame = frame.reshape((self.HEIGHT, self.WIDTH, 3))

        return frame
    
    def close(self):
        """
        Destroy the window and detach from the shared memory object
        """
        cv2.destroyAllWindows()
        self.shm.detach()



if __name__ == "__main__":
    cr = CameraRead()
    cr.get_camera_feed()