import serial
import time
import json
import firebase_admin
from firebase_admin import credentials, firestore

# Configura el puerto serial y la velocidad de baudios (asegúrate de que coincida con el de Arduino)
arduino_port = "COM4"  # Reemplaza con el puerto serial de tu Arduino
baud_rate = 9600

# Carga tus credenciales de Firebase
cred = credentials.Certificate("./vasitos-company-firebase-adminsdk-rnukz-6669b2d811.json")
firebase_admin.initialize_app(cred)

# Obtén una referencia a la base de datos Firestore
db = firestore.client()

def connect_serial():
    while True:
        try:
            ser = serial.Serial(arduino_port, baud_rate, timeout=1)
            print("Serial connection established.")
            return ser
        except serial.SerialException:
            print("Failed to connect to serial. Retrying in 10 seconds...")
            time.sleep(10)

def read_arduino(ser):
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            try:
                data = json.loads(line)
                return data
            except json.JSONDecodeError:
                return None
    except serial.SerialException:
        print("Device disconnected. Attempting to reconnect...")
        time.sleep(10)  # Wait before attempting to reconnect
        ser = connect_serial()
    return None

def send_ack():
    ack_payload = {"ACK": True}
    ack_json = json.dumps(ack_payload)
    ser.write(ack_json.encode())

def save_to_firebase(data):
    # Cambia "arduino_data" a la colección que quieras usar en Firestore
    doc_ref = db.collection("arduino_data").document()
    doc_ref.set(data)

try:
    ser = connect_serial()
    while True:
        sensor_data = read_arduino(ser)
        if sensor_data:
            print(f"Received from Arduino: {sensor_data}")
            save_to_firebase(sensor_data)
            # send_ack()
        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting program.")

finally:
    ser.close()  # Cierra la conexión serial
