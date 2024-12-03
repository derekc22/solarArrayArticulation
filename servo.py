import RPi.GPIO as GPIO  # Import GPIO control library for Raspberry Pi
import time  # For timing and delays
from random import randint  # For testing with random angles
import numpy as np  # For numerical operations and data handling
import matplotlib.pyplot as plt  # For plotting movement data

class Servo:
    def __init__(self, NUMBER, SERVO_PIN):
        self.NUMBER = NUMBER
        
        # Initialize servo with specified GPIO pin
        self.SERVO_PIN = SERVO_PIN  # Store GPIO pin number
        self.pwm = self.setPins()  # Setup PWM on specified pin

        self.curr_angle = 0  # Track current servo angle
        self.coord_file = open(f"data/servo{self.NUMBER}_coords.csv", "w+")  # Create file for logging movement data
        
    def setPins(self):
        # Configure GPIO pins for servo control
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)  # Set pin as output

        pwm = GPIO.PWM(self.SERVO_PIN, 50)  # Create PWM instance at 50Hz (standard servo frequency)
        pwm.start(0)  # Start PWM with 0% duty cycle (servo at rest)

        return pwm

    def writeData(self, t):
        # Log current time and angle to CSV file
        self.coord_file.write(f"\n{t},{self.curr_angle}")

    def plotData(self):
        # Load movement data from CSV file
        data = np.loadtxt(f"data/servo{self.NUMBER}_coords.csv", delimiter=',')

        # Extract time and position data
        time_data = data[:, 0]
        coord_data = data[:, 1]

        # Create movement plot
        plt.plot(time_data, coord_data)
        plt.xlabel('time [s]')
        plt.ylabel('pitch angle [deg]')
        plt.title(f"servo{self.NUMBER}: pitch angle vs time")

        # Save and display plot
        plt.savefig(f"data/servo{self.NUMBER}_coords_plt.pdf")
        plt.close()

    def moveFast(self, angle):
        # Quick movement to target angle (used for initialization)
        duty_cycle = (angle / 18) + 2  # Convert angle to duty cycle (servo standard conversion)
        self.pwm.ChangeDutyCycle(duty_cycle)  # Set new duty cycle
        time.sleep(0.8)  # Allow time for movement
        self.pwm.ChangeDutyCycle(0)  # Stop PWM to prevent jitter

        self.curr_angle = angle  # Update current angle

    def move(self, angle):
        # Smooth movement to target angle
        # Convert current and target angles to duty cycles
        curr_duty_cycle = int((self.curr_angle / 18) + 2)
        duty_cycle = int((angle / 18) + 2)

        # Determine direction of movement
        CCW = True if curr_duty_cycle < duty_cycle else False

        # Movement timing parameters
        pulse_width = 0.01  # Width of each PWM step
        sleep_time = 0.005  # Delay between steps

        # Create array of intermediate duty cycles for smooth movement
        increments = np.arange(curr_duty_cycle, duty_cycle+pulse_width, pulse_width) if CCW else np.arange(curr_duty_cycle, duty_cycle-pulse_width, -pulse_width)

        increment_count = len(increments)  # Total number of steps
        angle_increment = (angle-self.curr_angle)/increment_count  # Angle change per step

        # Execute movement in small steps
        for p in increments:
            self.pwm.ChangeDutyCycle(p)  # Set new duty cycle
            time.sleep(sleep_time)  # Brief pause
            self.curr_angle += angle_increment  # Update current angle

        self.pwm.ChangeDutyCycle(0)  # Stop PWM to prevent jitter

    def deploy(self):
        # Initialize servo to zero position
        self.moveFast(120)

    def cleanup(self):
        # Safely return servo to zero position in stages
        if self.curr_angle > 60:
            self.move(60)  # First stage if angle is very high
        if self.curr_angle > 30:
            self.move(30)  # Second stage if angle is moderately high
        self.move(0)  # Final move to zero position

        # Close log file and stop PWM
        self.coord_file.close()
        self.pwm.stop()

if __name__ == "__main__":
    # Test servo movement if run directly
    
    servo1 = Servo(1, 3)  # Create servo instance on pin 3
    servo1.moveFast(120)  # Move to test angle
    servo1.cleanup()  # Clean up
        
    servo2 = Servo(2, 40)  # Create servo instance on pin 2
    servo2.moveFast(120)  # Move to test angle
    servo2.cleanup()  # Clean up


