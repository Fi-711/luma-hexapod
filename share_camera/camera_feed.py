import cv2
import numpy as np
import sysv_ipc


class CameraFeed():
    """
    Class that creates a camera feed and shares the outputs in shared memory
    """
    def __init__(self, key=0xDEADBEEF, WIDTH=640, HEIGHT=480):
        # Key code for shared memory
        self.key = key
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT

        # Define the size of the shared memory buffer
        self.SHM_SIZE = WIDTH*HEIGHT*3

        # Create a shared memory object
        self.shm = sysv_ipc.SharedMemory(key=self.key, flags=sysv_ipc.IPC_CREAT, mode=0o666, size=self.SHM_SIZE)

        # Create a named window to display the camera feed
        # cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)

        # Resize the window to 640x480
        # cv2.resizeWindow("Camera Feed", WIDTH, HEIGHT)
        print("Sharing Camera Feed on key: 0xDEADBEEF")
        # Start the camera capture
        self.cap = cv2.VideoCapture(0)


    def share_camera_feed(self):
        """
        Share the camera feed into the memory so can be read by other scripts
        """
        while True:
            # Read a frame from the camera
            ret, frame = self.cap.read()

            # Convert the frame to a byte string
            frame_bytes = frame.tobytes()

            # Write the frame to shared memory
            self.shm.write(frame_bytes)

            # Display the frame in the window
            # cv2.imshow("Camera Feed", frame)
            
            # # Exit the loop if 'q' is pressed
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break


    def destroy_node(self):
        # Release the camera and destroy the window
        self.cap.release()
        cv2.destroyAllWindows()

        # Detach and remove the shared memory object
        self.shm.detach()
        # only have the feed script remove the shm
        self.shm.remove()   


if __name__ == "__main__":
    cf = CameraFeed()
    cf.share_camera_feed()
    cf.destroy_node()