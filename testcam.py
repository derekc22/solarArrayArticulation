# Import required libraries
import cv2 as cv  # OpenCV library for image processing
from picamera2 import Picamera2  # PiCamera2 library for Raspberry Pi camera control
import time



# Camera viewport settings
VIEWPORT_WIDTH = 4608  # Camera resolution width in pixels
VIEWPORT_HEIGHT = 2592  # Camera resolution height in pixels
VIEWPORT_FOV_WIDTH = 102  # Camera field of view width in degrees
VIEWPORT_FOV_HEIGHT = 67  # Camera field of view height in degrees


def initializeCamera():
    # Try to initialize the PiCamera2 with specified resolution and format
    try:
        picam2 = Picamera2()  # Create PiCamera2 instance
        cam_config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (VIEWPORT_WIDTH, VIEWPORT_HEIGHT)})  # Configure camera with specified resolution and RGB format
        picam2.configure(cam_config)  # Apply configuration
        picam2.start()  # Start camera
        print("Successfully initialized camera")
        return picam2
    except Exception as e:
        print(f"PiCamera2 initialization failed: {e}")




def main():
        camera = initializeCamera()
        time.sleep(2)
        
        while True:
                # Capture and process image
                init_frame = camera.capture_array()  # Capture raw frame
                flipped_frame = cv.flip(init_frame, 1)  # Flip image horizontally    
                
                cv.imshow("keypoints_frame", flipped_frame)

                if cv.waitKey(1) == 32:
                    break




if __name__ == "__main__":

        main()
