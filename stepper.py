import RPi.GPIO as GPIO  # Import GPIO control library for Raspberry Pi
import time  # For timing and delays
import numpy as np  # For numerical operations and data handling
import matplotlib.pyplot as plt  # For plotting movement data

class Stepper:
    def __init__(self, PUL_PIN, DIR_PIN, ENA_PIN, LIMIT_SWITCH_PIN):
        # Initialize stepper motor control pins
        self.PUL_PIN = PUL_PIN  # Pulse pin for step control
        self.DIR_PIN = DIR_PIN  # Direction pin for rotation direction
        self.ENA_PIN = ENA_PIN  # Enable pin for motor power
        self.LIMIT_SWITCH_PIN = LIMIT_SWITCH_PIN  # Pin for homing limit switch
        self.setPins()  # Configure all pins

        # Note: Pin mappings to stepper driver
        # CLK = PUL (Pulse pin)
        # CW = DIR (Direction pin)
        # EN = EN (Enable pin)

        self.curr_angle = 0  # Track current motor angle
        self.coord_file = open("stepper_coords.csv", "w+")  # Create file for logging movement data

    def setPins(self):
        # Configure GPIO pins for stepper control
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
        GPIO.setup(self.PUL_PIN, GPIO.OUT)  # Set pulse pin as output
        GPIO.setup(self.DIR_PIN, GPIO.OUT)  # Set direction pin as output
        GPIO.setup(self.ENA_PIN, GPIO.OUT)  # Set enable pin as output

        # Configure limit switch pin with pull-down resistor
        GPIO.setup(self.LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def writeData(self, t):
        # Log current time and angle to CSV file
        self.coord_file.write(f"\n{t},{self.curr_angle}")

    def plotData(self):
        # Load movement data from CSV file
        data = np.loadtxt('stepper_coords.csv', delimiter=',')

        # Extract time and position data
        time_data = data[:, 0]
        coord_data = data[:, 1]

        # Create movement plot
        plt.plot(time_data, coord_data)
        plt.xlabel('time [s]')
        plt.ylabel('pitch angle [deg]')
        plt.title('pitch angle vs time')

        # Save and display plot
        plt.savefig("stepper_coords_plt.pdf")
        plt.show()

    def move(self, angle):
        total_steps = 200  # Steps per full rotation (standard for most steppers)

        GPIO.output(self.ENA_PIN, GPIO.LOW)  # Enable the motor driver

        # Set direction based on angle sign
        direction = GPIO.HIGH if angle > 0 else GPIO.LOW
        # Calculate number of steps needed for desired angle
        steps = abs(int((total_steps/360)*angle))
        pulse_width = 1*0.01  # Width of step pulse

        GPIO.output(self.DIR_PIN, direction)  # Set movement direction

        # Execute movement one step at a time
        for _ in range(steps):
            GPIO.output(self.PUL_PIN, GPIO.HIGH)  # Begin step pulse
            time.sleep(pulse_width)  # Hold pulse
            GPIO.output(self.PUL_PIN, GPIO.LOW)  # End step pulse
            time.sleep(pulse_width)  # Delay between steps

            self.curr_angle += angle/steps  # Update current angle

        GPIO.output(self.ENA_PIN, GPIO.HIGH)  # Disable motor driver to save power

    def deploy(self):
        # Home the stepper motor using limit switch
        limit_switch = GPIO.input(self.LIMIT_SWITCH_PIN)

        # Move until limit switch is triggered
        while limit_switch == GPIO.LOW:
            limit_switch = GPIO.input(self.LIMIT_SWITCH_PIN)
            self.move(5)  # Move in small increments

        time.sleep(2)  # Pause at limit
        self.move(-30)  # Move to starting position

    def cleanup(self):
        # Return to zero position
        angle_to_move = -self.curr_angle  # Calculate angle needed to return to zero
        self.move(angle_to_move)  # Execute movement

        # Clean up resources
        self.coord_file.close()  # Close log file
        GPIO.output(self.ENA_PIN, GPIO.HIGH)  # Disable motor driver

if __name__ == "__main__":
    # Test stepper movement if run directly
    stepper1 = Stepper(10, 12, 8, 11)  # Create stepper instance with specified pins
    stepper1.move(180)  # Test 180-degree movement
    stepper1.cleanup()  # Clean up