import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BOARD)

# Set pin 11 as an output, and define as servo1 as PWM pin
GPIO.setup(11, GPIO.OUT)
servo1 = GPIO.PWM(11, 50)  # pin 11 for servo1, pulse 50Hz

# Start PWM running, with a value of 0 (pulse off)
servo1.start(0)

# Function to calculate the duty cycle for a given angle (in reverse)
def set_servo_angle(angle):
    # Reverse the angle mapping
    reversed_angle = 180 - angle
    duty = 2 + (reversed_angle / 18)  # Same duty cycle formula
    servo1.ChangeDutyCycle(duty)
    time.sleep(0.5)
    servo1.ChangeDutyCycle(0)

try:
    while True:
        # Ask user for angle and turn servo to it
        angle = float(input('Enter angle between 0 & 180: '))
        if 0 <= angle <= 180:
            set_servo_angle(angle)  # Use reversed angle
        else:
            print("Please enter a value between 0 and 180 degrees.")

finally:
    # Clean things up at the end
    servo1.stop()
    GPIO.cleanup()
    print("Goodbye!")
