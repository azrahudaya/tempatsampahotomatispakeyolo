# File: main.py
# Main Python controller for Smart Trash Bin on Raspberry Pi

import RPi.GPIO as GPIO
import time
import serial
import cv2
from ultralytics import YOLO
import threading
from RPLCD.i2c import CharLCD
import json
import logging

# Load config from config.json
def load_config(path="config.json"):
    with open(path, "r") as f:
        return json.load(f)

config = load_config()

# Logging setup
logging.basicConfig(
    filename="smart_trashbin.log",
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)

# config
kamera_index = config.get("kamera_index", 0)
batas_volume = config.get("batas_volume", 80)
port_serial = config.get("port_serial", "/dev/ttyUSB0")
baud_rate = config.get("baud_rate", 9600)

# Pin Configuration
pin_sensor_ir = config["pin_sensor_ir"]
pin_ultrasonik = config["pin_ultrasonik"]

# Setup GPIO
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    if isinstance(pin_sensor_ir, int):
        GPIO.setup(pin_sensor_ir, GPIO.IN)
    else:
        logging.error("pin_sensor_ir belum diisi nomor pin GPIO!")
        print("pin_sensor_ir belum diisi nomor pin GPIO!")
    for pin_set in pin_ultrasonik:
        if isinstance(pin_set["trigger"], int):
            GPIO.setup(pin_set["trigger"], GPIO.OUT)
        else:
            logging.error("trigger ultrasonik belum diisi nomor pin!")
            print("trigger ultrasonik belum diisi nomor pin!")
        if isinstance(pin_set["echo"], int):
            GPIO.setup(pin_set["echo"], GPIO.IN)
        else:
            logging.error("echo ultrasonik belum diisi nomor pin!")
            print("echo ultrasonik belum diisi nomor pin!")
except Exception as e:
    logging.exception("GPIO setup error")
    raise

# I2C LCD setup
lcd = CharLCD('PCF8574', 0x27)

def tampil_lcd(baris1="", baris2=""):
    try:
        lcd.clear()
        lcd.write_string(baris1[:16])
        lcd.cursor_pos = (1, 0)
        lcd.write_string(baris2[:16])
    except Exception as e:
        logging.error(f"lcd error: {str(e)}")
        print(f"lcd error: {str(e)}")

# Initialize YOLO model
model = YOLO("your_model_path.pt")  # model

# Serial Communication with Arduino
def konek_arduino():
    try:
        arduino = serial.Serial(port_serial, baud_rate, timeout=1)
        time.sleep(2)
        return arduino
    except Exception as e:
        tampil_lcd("arduino error", "tidak terhubung")
        logging.error(f"arduino connection error: {str(e)}")
        return None

arduino = konek_arduino()

# Category-to-position mapping
kategori = {
    "paper": 0,
    "plastic": 1,
    "metal": 2,
    "other": 3
}

# Helper Functions
def ukur_jarak(trigger_pin, echo_pin):
    try:
        if not (isinstance(trigger_pin, int) and isinstance(echo_pin, int)):
            logging.error("pin ultrasonik belum diisi nomor pin!")
            print("pin ultrasonik belum diisi nomor pin!")
            return 0
        GPIO.output(trigger_pin, True)
        time.sleep(0.00001)
        GPIO.output(trigger_pin, False)
        mulai, selesai = time.time(), time.time()
        while GPIO.input(echo_pin) == 0:
            mulai = time.time()
        while GPIO.input(echo_pin) == 1:
            selesai = time.time()
        durasi = selesai - mulai
        jarak = (durasi * 34300) / 2
        return jarak
    except Exception as e:
        logging.exception("ukur_jarak error")
        tampil_lcd("ultrasonik error", str(e)[:16])
        return 0

def cek_volume_chamber():
    volume = []
    for idx, pin in enumerate(pin_ultrasonik):
        try:
            jarak = ukur_jarak(pin['trigger'], pin['echo'])
            persen = max(0, min(100, 100 - (jarak / 20 * 100)))
            volume.append(persen)
            tampil_lcd(f"ch{idx+1} volume:", f"{persen:.1f}%")
            time.sleep(1)
            if persen >= batas_volume:
                tampil_lcd(f"ch{idx+1} penuh!", "segera kosongkan")
                logging.warning(f"Chamber {idx+1} penuh!")
                time.sleep(2)
        except Exception as e:
            tampil_lcd(f"ultrasonik {idx+1}", "error sensor")
            logging.exception(f"ultrasonik {idx+1} error")
            time.sleep(2)
    return volume

def tampil_error(pesan):
    logging.error(pesan)
    print(f"error: {pesan}")
    tampil_lcd("error!", pesan[:16])
    time.sleep(2)

def deteksi_objek(frame):
    try:
        hasil = model(frame)
        for res in hasil:
            for box in res.boxes:
                cls = res.names[int(box.cls)]
                if cls in kategori:
                    return cls
        return "other"
    except Exception as e:
        tampil_error(f"deteksi error: {str(e)}")
        logging.exception("deteksi_objek error")
        return "other"

def kirim_perintah_arduino(perintah):
    global arduino
    if arduino is None:
        arduino = konek_arduino()
        if arduino is None:
            tampil_error("arduino tidak terhubung")
            return False
    try:
        arduino.write((perintah + "\n").encode())
        time.sleep(0.1)
        timeout = time.time() + 3
        while time.time() < timeout:
            if arduino.in_waiting:
                feedback = arduino.readline().decode().strip()
                if feedback:
                    if feedback.startswith("ERR:"):
                        tampil_error(feedback[4:])
                        logging.error(f"Arduino error: {feedback[4:]}")
                        return False
                    elif feedback.startswith("OK:"):
                        tampil_lcd("arduino", feedback[3:19])
                        logging.info(f"Arduino OK: {feedback[3:19]}")
                        return True
        tampil_error("tidak ada respon arduino")
        logging.error("tidak ada respon arduino")
        return False
    except Exception as e:
        tampil_error("serial error")
        logging.exception("serial error")
        arduino = None
        return False

def gerak_chamber(kat):
    if kat in kategori:
        tampil_lcd("proses sortir", kat)
        if not kirim_perintah_arduino(f"GOTO:{kategori[kat]}"):
            return
        time.sleep(1)
        if not kirim_perintah_arduino("DROP"):
            return
        time.sleep(1)
        kirim_perintah_arduino("HOME")
    else:
        tampil_error(f"kategori tidak dikenal: {kat}")

def cek_sensor_ir():
    try:
        if not isinstance(pin_sensor_ir, int):
            logging.error("pin_sensor_ir belum diisi nomor pin GPIO!")
            print("pin_sensor_ir belum diisi nomor pin GPIO!")
            return
        if GPIO.input(pin_sensor_ir) == GPIO.HIGH:
            tampil_lcd("buka tutup...", "")
            if not kirim_perintah_arduino("OPEN_LID"):
                return
            time.sleep(2)
            kirim_perintah_arduino("CLOSE_LID")
            tampil_lcd("ambil gambar", "kamera...")
            cap = cv2.VideoCapture(kamera_index)
            ret, frame = cap.read()
            cap.release()
            if ret:
                kat = deteksi_objek(frame)
                tampil_lcd("terdeteksi:", kat)
                gerak_chamber(kat)
            else:
                tampil_error("kamera gagal ambil gambar")
                logging.error("kamera gagal ambil gambar")
    except Exception as e:
        tampil_error(f"sensor_ir error: {str(e)}")
        logging.exception("cek_sensor_ir error")

# Threading for volume check
def monitor_volume():
    while True:
        cek_volume_chamber()
        time.sleep(60)

thread_volume = threading.Thread(target=monitor_volume)
thread_volume.daemon = True
thread_volume.start()

# Main Loop
try:
    while True:
        cek_sensor_ir()
        time.sleep(0.5)
        if arduino and arduino.in_waiting:
            feedback = arduino.readline().decode().strip()
            if feedback:
                if feedback.startswith("ERR:"):
                    tampil_error(feedback[4:])
                    logging.error(f"Arduino error: {feedback[4:]}")
                elif feedback.startswith("OK:"):
                    tampil_lcd("arduino", feedback[3:19])
                    logging.info(f"Arduino OK: {feedback[3:19]}")
except KeyboardInterrupt:
    GPIO.cleanup()
    if arduino:
        arduino.close()
    lcd.clear()
    logging.info("program dihentikan oleh user")
    print("program dihentikan")
except Exception as e:
    logging.exception("Fatal error in main loop")
    GPIO.cleanup()
    if arduino:
        arduino.close()
    lcd.clear()
    print("program error, dihentikan")
