import cv2 # OpenCv for image processing
import sys # Exit scripts and accessing command lines
import requests # Requesting Datasets from Roboflow database 
import RPi.GPIO as GPIO # Raspberry Pi GPIO pins
from RpiMotorLib import RpiMotorLib # Stepper Motor
import time # Time Delays 
import threading # Multithreading 
from board import SCL, SDA # IC2 16 channer servo 
import busio # IC2 16 channer servo 
import psutil # Monitoring CPU memory 
from adafruit_pca9685 import PCA9685 # Controls the PCA9685 PWM driver (used for servos, LEDs)
from adafruit_servokit import ServoKit # High-level interface for servo control via PCA9685

print("âœ“ Script started")


# ================== GPIO SETUP ==================
GPIO.setmode(GPIO.BCM)S



# ================== ROBOFLOW MODEL CONFIGURATION ==================
API_KEY = "uyOn77A9VCQxMoZJkWEY"  # API key for Roboflow account that is currently used.
PROJECT_ID_CLASSIFICATION = "tomato_sorter-f6mb6" # Selecting which Type of Classification Project to use.
PROJECT_ID_OBJECTDETECTION = "tomato_detection-x39xt" # Selecting which Type of Object Detection Project to use. 
CLASSIFICATION_VERSION_ID = "4" # Dataset Version for Image Classificatoin Detection
OBJECTDETECTION_VERSION_ID = "1" # Dataset Version for Image Detection
CLASSIFICATION_URL = f"https://detect.roboflow.com/{PROJECT_ID_CLASSIFICATION}/{CLASSIFICATION_VERSION_ID}?api_key={API_KEY}&format=json" # Requesting RoboFlow
DETECTION_URL = f"https://detect.roboflow.com/{PROJECT_ID_OBJECTDETECTION}/{OBJECTDETECTION_VERSION_ID}?api_key={API_KEY}&format=json" # Requesting RoboFlow
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

class CameraThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.cap = None
        self.last_update = 0

    def run(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("? Error: Webcam not accessible")
            self.running = False
            return

        # Camera configuration
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 225)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 225)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Try to minimize buffer

        # Warm-up camera
        for _ in range(5):
            self.cap.read()

        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame.copy()
                    self.last_update = time.time()
            else:
                print("[ERROR] Camera thread failed to read frame")
            time.sleep(0.001)  # Reduce CPU usage

    def get_latest_frame(self):
        with self.lock:
            if self.frame is not None and (time.time() - self.last_update) < 0.5:
                return self.frame.copy()
            return None

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()




# Initialize and start camera thread
camera_thread = CameraThread()
camera_thread.start()
time.sleep(2)  # Allow camera to initialize


# ================== HARDWARE SETUP ==================
# Setting up 16 Channel Servokit
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
    'sort5': 6,
    'gate3': 12,
    'infer1':10
}


# Initialize servos by setting them to 0 then releasing (None)
for channel in SERVO_CHANNELS.values():
    kit.servo[channel].angle = 0
    kit.servo[channel].angle = None
    time.sleep(0.1)


# Setup stepper motor pins and initialize motor driver
step_motor_pins = [18, 23, 24, 25]
step_motor = RpiMotorLib.BYJMotor("MyMotor", "28BYJ")

def reset_sorting_servos():
    """Reset all sorting and gate servos to the closed position."""
    print("[ACTION] Resetting sorting gates...")
    for servo in ['gate1', 'gate2','gate3', 'sort1', 'sort2', 'sort3', 'sort4', 'sort5', 'infer1']:
        set_servo(servo, 0)
    print("[SUCCESS] Sorting gates reset")

def move_stepper(steps=700):
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

# ================== TOMATO SORTING FUNCTIONS ==================
def classify_tomato_size(real_width_cm):
    try:
        width = float(real_width_cm)
        print(f"[DEBUG] Width value: {real_width_cm}")
        
        
    except ValueError:
        print(f"[ERROR] Invalid width value: {real_width_cm}")
        return "normal"
        
        
    return "big" if width >= 6.55 else "normal" # filtering out detected tomato size to a standard measurement.

def process_inference(image_bytes, url, retries=3): # A repetion of 3 entries for a more valid image detection
    for attempt in range(retries):
        try:
            response = requests.post(url, files={"file": ("image.jpg", image_bytes, "image/jpeg")}, timeout=10)
            response.raise_for_status()
            return response.json()
            
        except requests.exceptions.RequestException as e:
            print(f"[ERROR] Attempt {attempt+1} failed: {e}")
            time.sleep(1)
            
    return None

# Defining servo and its rotation angle 

def set_servo(servo_name, angle): # defining servo and its rotation angle 
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
        
set_servo('infer1',0)
        
def open_gate3():
    set_servo('gate3', 65)
    time.sleep(0.13)  # Adjust speed of gate operation
    reset_sorting_servos()
    
def open_gate2():
    set_servo('gate2', 55)
    time.sleep(0.14)  # Adjust speed of gate operationSS
    reset_sorting_servos()
    
def open_gate1():
    set_servo('gate1', 45)
    time.sleep(0.13)  # Adjust speed of gate operation
    reset_sorting_servos()

# Initial servo adjustments for calibration

open_gate3()

time.sleep(1.5)

open_gate2()

time.sleep(1)
set_servo('gate1', 140)
move_stepper(steps=900) #initial steps of conveyor
time.sleep(0.5)

def detect_tomato():
    frame = camera_thread.get_latest_frame()
    if frame is None:
        print("[DEBUG] Frame capture failed.")
        return None

    # Process frame
    success, encoded_image = cv2.imencode('.jpg', frame)
    if not success:
        return None

    image_bytes = encoded_image.tobytes()
    
    # Perform classification inference
    classification_result = process_inference(image_bytes, CLASSIFICATION_URL)
    if not classification_result or 'top' not in classification_result:
        print("[DEBUG] Classification result is empty.")
        return None
    
    confidence = classification_result.get('confidence', 0)
    if confidence < 0.7:  # Require at least 70% confidence to classify Tomato into its designated category
        sys.stdout.write(f"[INFO] Low confidence ({confidence*100:.1f}%), skipping sorting.")
        sys.stdout.flush()
        return None

    predicted_class = classification_result['top'].lower()
    
    # Perform detection inference
    detection_result = process_inference(image_bytes, DETECTION_URL)
    if not detection_result or not detection_result.get("predictions"):
        print("[DEBUG] Detection result is empty.")
        return None
    
    # Check for multiple objects and if there are visible tomatoes
    predictions = detection_result["predictions"]
    
    detection_confidence = predictions[0].get("confidence", 0)
    if detection_confidence < 0.5: #object detection on 50 % confidence level, Does not really impact the classfication results
        sys.stdout.write(f"[OBJECT] Detection confidence ({detection_confidence*100:.1f}%) too low, skipping.")
        sys.stdout.flush()
        return None
    else:
        print(f"[INFO] Detection confidence ({detection_confidence*100:.1f}%)")

    # Calculate real tomato width from bounding box data
    bbox_width = predictions[0]["width"]
    real_width_cm = (bbox_width / 225) * 15 # 225 represents a knowSn real-world width and 15 is the real world size corresponding to that width
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
        ('red', 'big'): 2,
        ('green', 'big'): 3,
        ('red', 'normal'): 4,
        ('green', 'normal'): 5,
    }
    target_basket = class_mapping.get((predicted_class, size_category), 1)
    
    print(f"[ACTION] Sorting to basket {target_basket}")
    servo_name = f'sort{target_basket}'
    

    open_gate3()
    
    time.sleep(1.5)
    
    open_gate2()
    
    set_servo('infer1',140)
    set_servo(servo_name, 85)  # Open the sorting gate
    move_stepper(steps=1000)   # Move the conveyor to sort the tomato
    time.sleep(8) # 8 seconds time delay before the stepper starts movingSS
    reset_sorting_servos()
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
            
            # Tomato detected; process and reset the cooldown timer
            last_detection_time = time.time()
            sort_tomato(result['class'], result['size_category'])
        else:
            # Check if 45 seconds have passed without detection
            if time.time() - last_detection_time >= 45:
                print("[INFO] No tomato detected for 45 seconds. Resetting process.")
                reset_sorting_servos()
                open_gate3()
                
                time.sleep(1.5)
                
                open_gate2()
                
                time.sleep(3)
                set_servo('gate1', 140)
                last_detection_time = time.time()  # Reset cooldown

        # Move conveyor slightly between captures
        move_stepper(400)
        time.sleep(0.5)
        
# ================== MAIN LOOP CLEANUP ==================
def cleanup():
    camera_thread.stop() 
    GPIO.cleanup() #resets all GPIO pins that where recently used.
    cv2.destroyAllWindows() # closes any openCV GUI windows
    print("Resources released")


if __name__ == '__main__':
    try:
        main_loop()
    finally:
        cleanup()
