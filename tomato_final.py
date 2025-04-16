import cv2
import sys
import requests
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
import time
import threading
from board import SCL, SDA
import busio
import psutil
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

print("âœ“ Script started")

# ================== GPIO SETUP ==================
GPIO.setmode(GPIO.BCM)

# ================== ROBOFLOW MODEL CONFIGURATION ==================
API_KEY = "LTQH6YLQxTB3IhCZGgW3" #"NmTXUEpDtB1yKpd7rb8v"
PROJECT_ID_CLASSIFICATION = "tomato-sorter-pmz1z"#""tomato-sorter-gzogb
PROJECT_ID_OBJECTDETECTION = "tomato-sorter-od" #"" tomatosorter2025
CLASSIFICATION_VERSION_ID = "2"
OBJECTDETECTION_VERSION_ID = "1"
CLASSIFICATION_URL = f"https://detect.roboflow.com/{PROJECT_ID_CLASSIFICATION}/{CLASSIFICATION_VERSION_ID}?api_key={API_KEY}&format=json"
DETECTION_URL = f"https://detect.roboflow.com/{PROJECT_ID_OBJECTDETECTION}/{OBJECTDETECTION_VERSION_ID}?api_key={API_KEY}&format=json"
print("âœ“ Roboflow models configured")

# ================== CPU MONITORING ==================
def monitor_cpu_usage():
    """Continuously monitor and print the CPU usage. Warn if usage exceeds 80%."""
    while True:
        cpu_usage = psutil.cpu_percent(interval=1)
        sys.stdout.write(f"\r[DEBUG] CPU Usage: {cpu_usage:.2f}%   ")
        sys.stdout.flush()
        if cpu_usage > 80:
            print("\n[âš ï¸ WARNING] High CPU usage detected!")
        time.sleep(2)

cpu_thread = threading.Thread(target=monitor_cpu_usage, daemon=True)
cpu_thread.start()

# ================== CAMERA SETUP ==================
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("âœ– Error: Webcam not accessible")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 224)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 224)
cap.set(cv2.CAP_PROP_FPS, 30)

def capture_frame():
    for _ in range(5):
        cap.read()
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Failed to capture frame.")
        return None, None
    return ret, frame

# ================== HARDWARE SETUP ==================
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50
kit = ServoKit(channels=16)

# Define servo channel mapping for gates and sorters
SERVO_CHANNELS = {
    'gate1': 0,
    'gate2': 1,
    'sort1': 2,
    'sort2': 3,
    'sort3': 4,
    'sort4': 5,
    'sort5': 6
}

# Initialize servos by setting them to 0 then releasing (None)
for channel in SERVO_CHANNELS.values():
    kit.servo[channel].angle = 0
    kit.servo[channel].angle = None
    time.sleep(0.1)

# Setup stepper motor pins and initialize motor driver
step_motor_pins = [18, 23, 24, 25]
step_motor = RpiMotorLib.BYJMotor("MyMotor", "28BYJ")

def move_stepper(steps=512):
    """
    Move the stepper motor asynchronously to avoid blocking the main thread.
    :param steps: Number of steps to move the motor.
    """
    try:
        stepper_thread = threading.Thread(
            target=step_motor.motor_run,
            args=(step_motor_pins, 0.0008, steps, True, False, "half", 0.0008),
            daemon=True
        )
        stepper_thread.start()
    except Exception as e:
        print(f"[ERROR] Motor error: {str(e)}")
        


# ================== ULTRASONIC SENSOR CONFIGURATION ==================
# Sensor mapping: sensor_id -> (trig_pin, echo_pin)
SENSORS = {
    1: (16, 20),
    2: (21, 19),
    3: (26, 13),
    4: (11, 9),
    5: (6, 5)
}

BASKET_FULL_DISTANCE = 2  # Distance threshold (in cm) to consider a basket full

def setup_ultrasonic_sensors():
    """Configure GPIO pins for ultrasonic sensors."""
    for trig, echo in SENSORS.values():
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
        GPIO.output(trig, False)
    time.sleep(0.2)

setup_ultrasonic_sensors()

def get_distance(trig, echo):
    """
    Measure distance using an ultrasonic sensor.
    :param trig: Trigger pin number.
    :param echo: Echo pin number.
    :return: Distance in cm or None if timeout.
    """
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    timeout = time.time() + 0.1  # Max wait time of 0.1 seconds
    pulse_start = time.time()

    # Wait for the sensor to start the pulse
    while GPIO.input(echo) == 0 and time.time() < timeout:
        pulse_start = time.time()

    # Wait for the sensor to end the pulse
    while GPIO.input(echo) == 1 and time.time() < timeout:
        pulse_end = time.time()

    if time.time() > timeout:
        return None  # Sensor timeout

    return (pulse_end - pulse_start) * 17150  # A time difference to distance in cm

def check_basket_status(sensor_id):
    """Handle sensor errors gracefully"""
    try:
        trig, echo = SENSORS[sensor_id]
        readings = []
        
        for _ in range(5):
            dist = get_distance(trig, echo)
            print(f"[DEBUG] Sensor {sensor_id} raw reading: {dist} cm")
            if dist is not None:
                readings.append(dist)
            time.sleep(0.05)
        
        if not readings:
            print(f"[WARNING] Sensor {sensor_id} failed all readings")
            return False  # Assume basket is OK
            
        avg_distance = sum(readings) / len(readings)
        print(f"[DEBUG] Average basket distance: {avg_distance}cm")
        return avg_distance <= BASKET_FULL_DISTANCE
        
    except Exception as e:
        print(f"[ERROR] Basket check failed for sensor {sensor_id}: {str(e)}")
        return False  # Assume basket is OK

# ================== TOMATO SORTING FUNCTIONS ==================
def classify_tomato_size(real_width_cm):
    try:
        width = float(real_width_cm)
        print(f"[DEBUG] Width value: {real_width_cm}")
    except ValueError:
        print(f"[ERROR] Invalid width value: {real_width_cm}")
        return "normal"
    return "big" if width >= 8.3 else "normal"

def process_inference(image_bytes, url, retries=3):
    for attempt in range(retries):
        try:
            response = requests.post(url, files={"file": ("image.jpg", image_bytes, "image/jpeg")}, timeout=10)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"[ERROR] Attempt {attempt+1} failed: {e}")
            time.sleep(1)
    return None
        
    

def set_servo(servo_name, angle):
    """
    Move a specified servo to a given angle.
    :param servo_name: Name of the servo (as per SERVO_CHANNELS).
    :param angle: Target angle for the servo.
    """
    try:
        #print(f"[ACTION] Moving {servo_name} to {angle}Â°...")
        kit.servo[SERVO_CHANNELS[servo_name]].angle = 180 - angle
        time.sleep(0.05)
        #print(f"[SUCCESS] {servo_name} moved")
    except Exception as e:
        print(f"[ERROR] Servo error: {str(e)}")

# Initial servo adjustments for calibration
set_servo('gate2', 55)
time.sleep(0.13)  # Adjust speed of gate operation

def reset_sorting_servos():
    """Reset all sorting and gate servos to the closed position."""
    print("[ACTION] Resetting sorting gates...")
    for servo in ['gate1', 'gate2', 'sort1', 'sort2', 'sort3', 'sort4', 'sort5']:
        set_servo(servo, 0)
    print("[SUCCESS] Sorting gates reset")

reset_sorting_servos()
time.sleep(1)
set_servo('gate1', 140)
move_stepper(steps=300) #initial steps of conveyor


def detect_tomato():
    """
    Capture an image, perform classification and detection inferences,
    and determine tomato details such as class and size.
    :return: Dictionary containing tomato details or None if detection fails.
    """
    ret, frame = capture_frame()
    if not ret or frame is None:
        print("[DEBUG] Frame capture failed.")
        return None

    # Resize frame and encode to JPEG for inference
    success, encoded_image = cv2.imencode('.jpg', cv2.resize(frame, (224, 224)))
    if not success:
        return None

    image_bytes = encoded_image.tobytes()
    
    # Perform classification inference
    classification_result = process_inference(image_bytes, CLASSIFICATION_URL)
    if not classification_result or 'top' not in classification_result:
        print("[DEBUG] Classification result is empty.")
        return None
    
    confidence = classification_result.get('confidence', 0)
    if confidence < 0.7:  # Require at least 70% confidence
        print(f"[INFO] Low confidence ({confidence*100:.1f}%), skipping sorting.")
        return None

    predicted_class = classification_result['top'].lower()
    
    # Perform detection inference
    detection_result = process_inference(image_bytes, DETECTION_URL)
    if not detection_result or not detection_result.get("predictions"):
        print("[DEBUG] Detection result is empty.")
        return None
    
    # Check for multiple objects
    predictions = detection_result["predictions"]
    if len(predictions) >= 3:
        print("[INFO] Multiple tomatoes detected in the image.")
        return {'multi_objects': True}
    
    detection_confidence = predictions[0].get("confidence", 0)
    if detection_confidence < 0.6: #object detection
        print(f"[INFO] Detection confidence ({detection_confidence*100:.1f}%) too low, skipping.")
        return None
    else:
        print(f"[INFO] Detection confidence ({detection_confidence*100:.1f}%)")

    # Calculate real tomato width from bounding box data
    bbox_width = predictions[0]["width"]
    real_width_cm = (bbox_width / 224) * 15
    tomato_size = classify_tomato_size(real_width_cm)

    result = {
        "class": predicted_class,
        "size_category": tomato_size,
        "confidence": confidence,
        "detection_confidence": detection_confidence
    }
    print(f"\nâœ“ TOMATO DETECTED âœ“")
    print(f"| Class: {predicted_class}")
    print(f"| Size: {tomato_size}")
    print(f"| Classification Confidence: {confidence*100:.1f}%")
    print(f"| Detection Confidence: {detection_confidence*100:.1f}%")
    print(f"| Real Width: {real_width_cm:.2f}cm")
    
    return result
    
def sort_tomato(predicted_class, size_category):
    """
    Sort the tomato based on its predicted class and size.
    :param predicted_class: The class (color/quality) predicted by the model.
    :param size_category: The size category ("big" or "normal") determined from detection.
    """
    # Mapping from (predicted_class, size_category) to basket number
    class_mapping = {
        ('bad', 'normal'): 1,
        ('green', 'normal'): 2,
        ('red', 'normal'): 3,
        ('red', 'big'): 4,
        ('green', 'big'): 5,
    }
    target_basket = class_mapping.get((predicted_class, size_category), 3)
    
    # Check if the target basket is full
    if check_basket_status(target_basket):
        print(f"[ACTION] Skipping full basket {target_basket}")
        move_stepper(steps=512)
        return

    print(f"[ACTION] Sorting to basket {target_basket}")
    servo_name = f'sort{target_basket}'
    
    set_servo('gate2', 45) #Revision, drop new tomato if there's a detected tomato.
    time.sleep(0.13)
    reset_sorting_servos()
    
    set_servo(servo_name, 80)  # Open the sorting gate
    move_stepper(steps=1000)   # Move the conveyor to sort the tomato
    time.sleep(8)
    set_servo(servo_name,0)
    set_servo('gate1', 140)


# ================== MAIN LOOP ==================
def main_loop():
    """
    Continuously capture tomatoes and process them for sorting.
    If no tomato is detected within 30 seconds, reset the process.
    """
    last_detection_time = time.time()  # Track the time of last tomato detection

    while True:
        print("\n[INFO] Capturing new tomato for sorting...")
        result = detect_tomato()
        
        if result:
            # Handle multiple objects case
            if 'multi_objects' in result:
                print("[ACTION] Multiple tomatoes detected - skipping sorting")
                move_stepper(steps=1024)  # Move conveyor to bypass sorting
                last_detection_time = time.time()
                time.sleep(2)
                set_servo('gate2',45)
                time.sleep(0.13)
                reset_sorting_servos()
                time.sleep(2)
                set_servo('gate1',140)
                continue
                
            
            # Tomato detected; process and reset the cooldown timer
            last_detection_time = time.time()
            sort_tomato(result['class'], result['size_category'])
        else:
            # Check if 40 seconds have passed without detection
            if time.time() - last_detection_time >= 40:
                print("[INFO] No tomato detected for 45 seconds. Resetting process.")
                reset_sorting_servos()
                set_servo('gate2',40)
                time.sleep(0.13)
                set_servo('gate2',0)
                time.sleep(3)
                set_servo('gate1', 140)
                last_detection_time = time.time()  # Reset cooldown

        # Move conveyor slightly between captures
        move_stepper(150)
        time.sleep(0.1)

if __name__ == '__main__':
    main_loop()
