import time
import cv2
from picamera2 import Picamera2
import roboflow
import os
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib

print("Script started")

# Initialize Roboflow with your API key
rf = roboflow.Roboflow(api_key="LTQH6YLQxTB3IhCZGgW3")
print("Roboflow initialized")

# Access the project and model
project = rf.workspace().project("tomato-sorter-pmz1z")
model = project.version("2").model
print("Project and model loaded")

# Initialize the PiCamera2
picam2 = Picamera2()
print("PiCamera2 initialized")

# Configure the camera for live streaming (640x480 resolution)
camera_config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(camera_config)
print("Camera configured")

# Start the camera
picam2.start()
print("Camera started")

# Setup GPIO to use BCM mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  # Disable warnings
print("GPIO mode set to BCM")

# Servo motor pins for the gates (using BCM numbering)
servo_pin_1 = 17  # Gate 1 (Red tomato)
servo_pin_2 = 27  # Gate 2 (Green tomato)
servo_pin_3 = 22  # Gate 3 (Bad tomato)

# Initialize serv000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000os
GPIO.setup(servo_pin_1, GPIO.OUT)
GPIO.setup(servo_pin_2, GPIO.OUT)
GPIO.setup(servo_pin_3, GPIO.OUT)
print("GPIO pins set up")

# Setup PWM for servos (50Hz)
servo_1 = GPIO.PWM(servo_pin_1, 50)
servo_2 = GPIO.PWM(servo_pin_2, 50)
servo_3 = GPIO.PWM(servo_pin_3, 50)
print("PWM for servos set up")

# Start the servos at a neutral position (closed gates)
servo_1.start(0)
servo_2.start(0)
servo_3.start(0)
print("Servos started at neutral position")

# Define stepper motor pins (using BCM numbering)
step_motor_pins = [18, 23, 24, 25]  # Replace these with your actual BCM pin numbers
print("Stepper motor pins defined")

# Initialize the stepper motor using GPIO.BCM mode
step_motor = RpiMotorLib.BYJMotor("MyMotor", "28BYJ")
print("Stepper motor initialized")

# Path to temporarily save images for inference
temp_image_path = "/tmp/live_frame.jpg"

def move_stepper(steps=512):
    """Move the stepper motor a certain number of steps."""
    print(f"Moving stepper motor {steps} steps")
    step_motor.motor_run(step_motor_pins, 0.001, steps, True, False, "half", 0.001)
    print("Stepper motor movement complete")


def set_servo_angle(servo, angle):
    """Open the specified gate by turning the servo to the specified angle (reversed)."""
    reversed_angle = 180 - angle  # Reverse the angle
    print(f"Opening gate to {reversed_angle} degrees")
    duty = 2 + (reversed_angle / 18)  # Calculate the duty cycle for the angle
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)  # Stop sending signal to avoid jitter
    print("Gate opened")

try:
    print("Entering main loop")
    while True:
        print("Capturing frame...")
        # Capture a frame from the camera
        frame = picam2.capture_array()
        print("Frame captured")

        # Save the frame temporarily
        cv2.imwrite(temp_image_path, frame)
        print(f"Frame saved to {temp_image_path}")

        # Ensure the temporary file was written
        if not os.path.exists(temp_image_path):
            print("Error: Image capture failed.")
            continue

        print("Running inference...")
        # Run inference on the saved frame
        try:
            predictions = model.predict(temp_image_path).json()
            print("Inference complete")
        except Exception as e:
            print(f"Inference failed: {e}")
            continue

        # Check if there are any predictions
        if 'predictions' in predictions and predictions['predictions']:
            found_objects = False
            # Iterate through predictions
            for prediction_set in predictions['predictions']:
                for prediction in prediction_set['predictions']:
                    # Only show predictions with confidence >= 70%
                    confidence = prediction['confidence']
                    if confidence >= 0.50:
                        class_name = prediction['class']
                        print(f" - {class_name} (Confidence: {confidence:.2f})")
                        found_objects = True

                        # Step 1: Move tomato to the sorting area (using stepper motor)
                        print("Moving tomato to sorting area...")
                        move_stepper()

                        # Step 2: Open the correct gate based on the tomato's classification
                        if class_name == "Red":
                            print("Opening Gate 1 (Red)...")
                            set_servo_angle(servo_1, 60)  # Open gate 1
                        elif class_name == "Green":
                            print("Opening Gate 2 (Green)...")
                            set_servo_angle(servo_2, 60)  # Open gate 2
                        elif class_name == "Bad":
                            print("Opening Gate 3 (Bad)...")
                            set_servo_angle(servo_3, 60)  # Open gate 3
                        
                        # Step 3: Delay and then close the gate
                        time.sleep(2)  # Allow time for tomato to drop
                        print("Closing all gates...")
                        set_servo_angle(servo_1, 0)  # Close gate 1
                        set_servo_angle(servo_2, 0)  # Close gate 2
                        set_servo_angle(servo_3, 0)  # Close gate 3

            # If no high-confidence object was found, display a message
            if not found_objects:
                print("No high-confidence objects detected (confidence >= 50%).")
        else:
            # If no objects are detected at all
            print("No objects detected.")

        # Optional: Delay to avoid overwhelming the system with logs (adjust as needed)
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Script interrupted by user")
finally:
    # Stop the camera and clean up
    picam2.stop()
    servo_1.stop()
    servo_2.stop()
    servo_3.stop()
    GPIO.cleanup()
    print("Camera and motors stopped. Script ended.")
