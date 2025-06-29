# File: main_controller.py
# Main Python controller for Smart Trash Bin on Raspberry Pi

import RPi.GPIO as GPIO
import time
import serial
import cv2
from ultralytics import YOLO
import threading
from RPLCD.i2c import CharLCD

# Configuration
CAMERA_INDEX = 0
VOLUME_THRESHOLD = 80  # percent
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600

# Pin Configuration
IR_SENSOR_PIN = "PINXX"  # Define the actual GPIO pin
ULTRASONIC_PINS = [
    {"trigger": "PINXX", "echo": "PINXX"},  # Chamber 1
    {"trigger": "PINXX", "echo": "PINXX"},  # Chamber 2
    {"trigger": "PINXX", "echo": "PINXX"},  # Chamber 3
    {"trigger": "PINXX", "echo": "PINXX"},  # Chamber 4
]

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(IR_SENSOR_PIN, GPIO.IN)
for pin_set in ULTRASONIC_PINS:
    GPIO.setup(pin_set["trigger"], GPIO.OUT)
    GPIO.setup(pin_set["echo"], GPIO.IN)

# I2C LCD setup
lcd = CharLCD('PCF8574', 0x27)

def lcd_display(line1="", line2=""):
    try:
        lcd.clear()
        lcd.write_string(line1[:16])
        lcd.cursor_pos = (1, 0)
        lcd.write_string(line2[:16])
    except Exception as e:
        print(f"lcd error: {str(e)}")

# Initialize YOLO model
model = YOLO("your_model_path.pt")  # Your YOLOv8 model path

# Serial Communication with Arduino
def connect_arduino():
    try:
        arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        return arduino
    except Exception as e:
        lcd_display("arduino error", "tidak terhubung")
        return None

arduino = connect_arduino()

# Category-to-position mapping
CATEGORIES = {
    "paper": 0,
    "plastic": 1,
    "metal": 2,
    "other": 3
}

# Helper Functions
def measure_distance(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)
    start, stop = time.time(), time.time()

    while GPIO.input(echo_pin) == 0:
        start = time.time()
    while GPIO.input(echo_pin) == 1:
        stop = time.time()

    elapsed = stop - start
    distance = (elapsed * 34300) / 2
    return distance

def get_chambers_volume():
    volumes = []
    for idx, pin in enumerate(ULTRASONIC_PINS):
        try:
            dist = measure_distance(pin['trigger'], pin['echo'])
            volume_percent = max(0, min(100, 100 - (dist / 20 * 100)))
            volumes.append(volume_percent)
            lcd_display(f"ch{idx+1} volume:", f"{volume_percent:.1f}%")
            time.sleep(1)
            if volume_percent >= VOLUME_THRESHOLD:
                lcd_display(f"ch{idx+1} penuh!", "segera kosongkan")
                time.sleep(2)
        except Exception as e:
            lcd_display(f"ultrasonik {idx+1}", "error sensor")
            time.sleep(2)
    return volumes

def show_error(error_msg):
    print(f"error: {error_msg}")
    lcd_display("error!", error_msg[:16])
    time.sleep(2)

def detect_object(frame):
    results = model(frame)
    for result in results:
        for box in result.boxes:
            cls = result.names[int(box.cls)]
            if cls in CATEGORIES:
                return cls
    return "other"

def send_arduino_cmd(cmd):
    global arduino
    if arduino is None:
        arduino = connect_arduino()
        if arduino is None:
            show_error("arduino tidak terhubung")
            return False
    try:
        arduino.write((cmd + "\n").encode())
        time.sleep(0.1)
        # baca feedback dari arduino
        timeout = time.time() + 3
        while time.time() < timeout:
            if arduino.in_waiting:
                feedback = arduino.readline().decode().strip()
                if feedback:
                    if feedback.startswith("ERR:"):
                        show_error(feedback[4:])
                        return False
                    elif feedback.startswith("OK:"):
                        lcd_display("arduino", feedback[3:19])
                        return True
        show_error("tidak ada respon arduino")
        return False
    except Exception as e:
        show_error("serial error")
        arduino = None
        return False

def move_chamber_to(category):
    if category in CATEGORIES:
        lcd_display("proses sortir", category)
        if not send_arduino_cmd(f"GOTO:{CATEGORIES[category]}"):
            return
        time.sleep(1)
        if not send_arduino_cmd("DROP"):
            return
        time.sleep(1)
        send_arduino_cmd("HOME")
    else:
        show_error(f"kategori tidak dikenal: {category}")

def ir_sensor_callback():
    if GPIO.input(IR_SENSOR_PIN) == GPIO.HIGH:
        lcd_display("buka tutup...", "")
        if not send_arduino_cmd("OPEN_LID"):
            return
        time.sleep(2)
        send_arduino_cmd("CLOSE_LID")
        lcd_display("ambil gambar", "kamera...")

        cap = cv2.VideoCapture(CAMERA_INDEX)
        ret, frame = cap.read()
        cap.release()
        if ret:
            category = detect_object(frame)
            lcd_display("terdeteksi:", category)
            move_chamber_to(category)
        else:
            show_error("kamera gagal ambil gambar")

# Threading for volume check
def volume_monitor():
    while True:
        get_chambers_volume()
        time.sleep(60)

volume_thread = threading.Thread(target=volume_monitor)
volume_thread.daemon = True
volume_thread.start()

# Main Loop
try:
    while True:
        ir_sensor_callback()
        time.sleep(0.5)
        # cek feedback dari arduino jika ada
        if arduino and arduino.in_waiting:
            feedback = arduino.readline().decode().strip()
            if feedback:
                if feedback.startswith("ERR:"):
                    show_error(feedback[4:])
                elif feedback.startswith("OK:"):
                    lcd_display("arduino", feedback[3:19])
except KeyboardInterrupt:
    GPIO.cleanup()
    if arduino:
        arduino.close()
    lcd.clear()
    print("program dihentikan")
