# Import required libraries
import cv2 as cv  # OpenCV library for image processing
import numpy as np  # NumPy for numerical operations
import CLINGINIT  # Custom module for blob detection initialization
import time  # For timing and delays
from picamera2 import Picamera2  # PiCamera2 library for Raspberry Pi camera control
from servo import Servo  # Custom Servo motor control class
from stepper import Stepper  # Custom Stepper motor control class
import math  # For mathematical operations
from concurrent.futures import ThreadPoolExecutor  # For parallel motor control
from PIL import Image  # For image saving operations
import os  # For file and directory operations

# Image processing constants
BINARY_THRESH = 254  # Threshold value for binary image conversion (high value to isolate bright spots)
GAUSS_BLUR = 25  # Gaussian blur kernel size for noise reduction

# Camera viewport settings
VIEWPORT_WIDTH = 4608  # Camera resolution width in pixels
VIEWPORT_HEIGHT = 2592  # Camera resolution height in pixels
VIEWPORT_FOV_WIDTH = 102  # Camera field of view width in degrees
VIEWPORT_FOV_HEIGHT = 67  # Camera field of view height in degrees

# Calculate viewport midpoints for sun position normalization
VIEWPORT_WIDTH_MIDPOINT = VIEWPORT_WIDTH / 2  # Center x-coordinate of viewport
VIEWPORT_HEIGHT_MIDPOINT = VIEWPORT_HEIGHT / 2  # Center y-coordinate of viewport

# Motor movement limits
MAX_STEPPER_ANGLE = 30  # Maximum rotation angle for stepper motors (degrees)
MAX_SERVO_ANGLE = 120  # Maximum angle for servo motors (degrees)
MIN_SERVO_ANGLE = 60  # Minimum angle for servo motors (degrees)

# Minimum angle change thresholds to prevent unnecessary motor movement
SERVO_ANGLE_CHANGE_THRESHOLD = 0.1  # Minimum angle change to trigger servo movement
STEPPER_ANGLE_CHANGE_THRESHOLD = 0.1  # Minimum angle change to trigger stepper movement

CAMERA_ANGLE_TO_SERVO_ANGLE_OFFSET = 90 # DONT CHANGE
CAMERA_ANGLE_TO_STEPPER_ANGLE_OFFSET = 0 # DONT CHANGE

# Initialize primary solar panel motor control pins
NUMBER1 = 1
SERVO1_PIN = 40 # Store GPIO pin number
STEPPER1_PUL_PIN = 35 # Pulse pin for step control
STEPPER1_DIR_PIN = 37 # Direction pin for rotation direction
STEPPER1_ENA_PIN = 38 # Enable pin for motor power
STEPPER1_LIMIT_SWITCH_PIN = 11 # Pin for homing limit switch

# Initialize secondary solar panel motor control pins
NUMBER2 = 2
SERVO2_PIN = 3 # Store GPIO pin number
STEPPER2_PUL_PIN = 10 # Pulse pin for step control
STEPPER2_DIR_PIN = 12 # Direction pin for rotation direction
STEPPER2_ENA_PIN = 8 # Enable pin for motor power
STEPPER2_LIMIT_SWITCH_PIN = 13 # Pin for homing limit switch

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

def processFrame(camera, detector):
    # Capture and process image
    init_frame = camera.capture_array()  # Capture raw frame
    flipped_frame = cv.flip(init_frame, 1)  # Flip image horizontally
    frame = cv.cvtColor(flipped_frame, cv.COLOR_BGR2GRAY)  # Convert to grayscale
    _, frame = cv.threshold(frame, BINARY_THRESH, 255, cv.THRESH_BINARY)  # Convert to binary
    frame = cv.medianBlur(frame, GAUSS_BLUR)  # Apply blur to reduce noise

    # Detect bright spots (sun) in image
    keypoints = detector.detect(frame)
    sorted_keypoints = sorted(keypoints, key=lambda k: k.size, reverse=True)  # Sort by size
    keypoints_frame = cv.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)  # Draw detected points

    return sorted_keypoints, keypoints_frame

def recordMotorData(threads, motors, t):
    # Continue recording while any motor movement thread is still active
    if not threads:
        for motor in motors:
            motor.writeData(t)
        return

    while any(not thread.done() for thread in threads):
        for motor in motors:
            motor.writeData(time.time() - t)  # Record current motor position and time
        time.sleep(0.02)  # Short delay between recordings

def saveImage(keypoints_frame, motors, elapsed_time):
    # Unpack motor objects
    servo1, stepper1, servo2, stepper2 = motors

    # Convert frame to PIL Image format
    image_to_save = Image.fromarray(keypoints_frame)
    os.makedirs('../seniordesign 2/photos', exist_ok=True)  # Create photos directory if it doesn't exist
    # Save image with timestamp and current motor angles in filename
    image_to_save.save(f"photos/keypointsframe_{elapsed_time}_{servo1.curr_angle: .5f}_{stepper1.curr_angle: .5f}_{servo2.curr_angle: .5f}_{stepper2.curr_angle: .5f}.jpg")

def checkServoLimits(pitch_angle_setpoint):
    # Ensure servo angle stays within defined limits
    if pitch_angle_setpoint > MAX_SERVO_ANGLE:
        print("max pitch angle exceeded!")
        pitch_angle_setpoint = MAX_SERVO_ANGLE
    elif pitch_angle_setpoint < MIN_SERVO_ANGLE:
        print("min pitch angle exceeded!")
        pitch_angle_setpoint = MIN_SERVO_ANGLE

    return pitch_angle_setpoint

def checkStepperLimits(yaw_angle_setpoint, stepper):
    # Ensure stepper angle stays within defined limits
    if abs(stepper.curr_angle + yaw_angle_setpoint) > MAX_STEPPER_ANGLE:
        print("max yaw angle exceeded!")
        yaw_angle_setpoint = math.copysign(MAX_STEPPER_ANGLE, yaw_angle_setpoint) - stepper.curr_angle

    return yaw_angle_setpoint

def checkAngleChangeThreshold(motor, setpoint):
    # Check if angle change is significant enough to warrant movement
    if isinstance(motor, Servo):
        return abs(abs(setpoint) - motor.curr_angle) > SERVO_ANGLE_CHANGE_THRESHOLD
    elif isinstance(motor, Stepper):
        return abs(setpoint) > STEPPER_ANGLE_CHANGE_THRESHOLD

def initializeMotors():
    # Initialize all four motors with their respective GPIO pins
    servo1 = Servo(NUMBER1, SERVO1_PIN)  # Primary pitch control servo
    stepper1 = Stepper(NUMBER1, STEPPER1_PUL_PIN, STEPPER1_DIR_PIN, STEPPER1_ENA_PIN, STEPPER1_LIMIT_SWITCH_PIN)  # Primary yaw control stepper

    servo2 = Servo(NUMBER2, SERVO2_PIN)  # Secondary pitch control servo
    stepper2 = Stepper(NUMBER2, STEPPER2_PUL_PIN, STEPPER2_DIR_PIN, STEPPER2_ENA_PIN, STEPPER2_LIMIT_SWITCH_PIN)  # Secondary yaw control stepper

    # Create dictionary of motors for easy access
    motors = {
        "servo1": servo1,
        "stepper1": stepper1,
        "servo2": servo2,
        "stepper2": stepper2,
    }

    return motors

def calculateSetpoints(sunXNorm, sunYNorm, stepper1, stepper2):
    # Calculate setpoints for primary solar panel (Servo1, Stepper1)
    camera_angle_setpoint_y = (sunYNorm / VIEWPORT_HEIGHT) * VIEWPORT_FOV_HEIGHT  # Convert pixel position to angle
    camera_angle_setpoint_x = (sunXNorm / VIEWPORT_WIDTH) * VIEWPORT_FOV_WIDTH

    """PANEL 1"""
    # https://www.desmos.com/calculator/1xxxjtoix8
    pitch_angle_setpoint1 = -((MAX_SERVO_ANGLE-MIN_SERVO_ANGLE)/VIEWPORT_FOV_HEIGHT) * camera_angle_setpoint_y + CAMERA_ANGLE_TO_SERVO_ANGLE_OFFSET
    # https://www.desmos.com/calculator/rljl6r6bk0
    if pitch_angle_setpoint1 > 90:
        yaw_angle_setpoint1 = -((2*MAX_STEPPER_ANGLE/VIEWPORT_FOV_WIDTH) * camera_angle_setpoint_x + CAMERA_ANGLE_TO_STEPPER_ANGLE_OFFSET) - stepper1.curr_angle
    elif pitch_angle_setpoint1 <= 90:
        yaw_angle_setpoint1 = ((2*MAX_STEPPER_ANGLE/VIEWPORT_FOV_WIDTH) * camera_angle_setpoint_x + CAMERA_ANGLE_TO_STEPPER_ANGLE_OFFSET) - stepper1.curr_angle
        
    # Ensure angles are within limits
    pitch_angle_setpoint1 = checkServoLimits(pitch_angle_setpoint1)
    yaw_angle_setpoint1 = checkStepperLimits(yaw_angle_setpoint1, stepper1)

    """PANEL 2"""
    # https://www.desmos.com/calculator/42dgs3tl8p
    # Calculate setpoints for secondary solar panel (Servo2, Stepper2)
    pitch_angle_setpoint2 = 180 - pitch_angle_setpoint1 # Convert primary pitch to secondary pitch
    yaw_angle_setpoint2 = yaw_angle_setpoint1

    # Ensure angles are within limits
    pitch_angle_setpoint2 = checkServoLimits(pitch_angle_setpoint2)
    yaw_angle_setpoint2 = checkStepperLimits(yaw_angle_setpoint2, stepper2)

    return pitch_angle_setpoint1, yaw_angle_setpoint1, pitch_angle_setpoint2, yaw_angle_setpoint2

def main():
    # Get blob detector from CLINGINIT module
    detector = CLINGINIT.detector

    # Initialize and deploy all motors
    motors = initializeMotors().values()
    servo1, stepper1, servo2, stepper2 = motors

    for motor in motors:
        motor.deploy()  # Move motors to starting positions
    
    # Initialize camera and wait for warm-up
    camera = initializeCamera()
    time.sleep(2)

    start_time = time.time()  # Record start time for timing operations
    
    iterations = 0
    try:
        # Create thread pool for parallel motor control
        with ThreadPoolExecutor(max_workers=4) as executor:

            while True:
                # Capture and process image
                sorted_keypoints, keypoints_frame = processFrame(camera, detector)

                # Reset threads
                threads = None

                # If sun is detected, adjust panel positions
                if sorted_keypoints:
                    print("-------------------------------------------------------------------------------------------")

                    sun = sorted_keypoints[0]  # Get largest bright spot (assumed to be sun)
                    sunX, sunY = sun.pt  # Get sun coordinates

                    # Calculate normalized sun position relative to viewport center
                    sunXNorm = sunX - VIEWPORT_WIDTH_MIDPOINT
                    sunYNorm = sunY - VIEWPORT_HEIGHT_MIDPOINT

                    # Calculate new motor positions
                    setpoints = calculateSetpoints(sunXNorm, sunYNorm, stepper1, stepper2)

                    # Move motors in parallel if angle change is significant
                    threads = [executor.submit(motor.move, setpoint) for (motor, setpoint) in zip(motors, setpoints) if checkAngleChangeThreshold(motor, setpoint)]


                elapsed_time = time.time() - start_time  # Calculate elapsed time

                # Record motor positions during movement
                recordMotorData(threads=threads, motors=motors, t=start_time if threads else elapsed_time)

                # Print current motor positions
                print(f"time: {elapsed_time}, servo1.curr_angle: {servo1.curr_angle: .5f}, stepper1.curr_angle: {stepper1.curr_angle: .5f}, servo2.curr_angle: {servo2.curr_angle: .5f}, stepper2.curr_angle: {stepper2.curr_angle: .5f}")

                # Display processed image
                cv.imshow("keypoints_frame", keypoints_frame)

                # Save image periodically
                iterations += 1
                if iterations % 5 == 0:
                    saveImage(keypoints_frame, motors, elapsed_time)

                # Check for spacebar press to exit
                if cv.waitKey(1) == 32:
                    break

    except Exception as e:
        print(f"An error occurred: {e}")
        raise e
        
    finally:
        # Clean up resources
        if 'camera' in locals():
            camera.stop()
            
        cv.destroyAllWindows()

        # Return motors to home positions
        for motor in reversed(motors):
            motor.cleanup()

        # Plot motor movement data
        for motor in motors:
            motor.plotData()

# Run main function if script is executed directly
if __name__ == "__main__":
    main()
