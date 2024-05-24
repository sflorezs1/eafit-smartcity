import serial
import time
import json
import firebase_admin
from firebase_admin import credentials, firestore
import google.generativeai as genai
import os

# Configura el puerto serial y la velocidad de baudios (asegúrate de que coincida con el de Arduino)
arduino_port = "COM4"  # Reemplaza con el puerto serial de tu Arduino
baud_rate = 9600

# Carga tus credenciales de Firebase
cred = credentials.Certificate("./vasitos-company-firebase-adminsdk-rnukz-6669b2d811.json")
firebase_admin.initialize_app(cred)

# Obtén una referencia a la base de datos Firestore
db = firestore.client()

genai.configure(api_key="")
generation_config = {
  "temperature": 1,
  "top_p": 0.95,
  "top_k": 64,
  "max_output_tokens": 8192,
  "response_mime_type": "text/plain",
}
safety_settings = [
  {
    "category": "HARM_CATEGORY_HARASSMENT",
    "threshold": "BLOCK_MEDIUM_AND_ABOVE",
  },
  {
    "category": "HARM_CATEGORY_HATE_SPEECH",
    "threshold": "BLOCK_MEDIUM_AND_ABOVE",
  },
  {
    "category": "HARM_CATEGORY_SEXUALLY_EXPLICIT",
    "threshold": "BLOCK_MEDIUM_AND_ABOVE",
  },
  {
    "category": "HARM_CATEGORY_DANGEROUS_CONTENT",
    "threshold": "BLOCK_MEDIUM_AND_ABOVE",
  },
]

model = genai.GenerativeModel(
  model_name="gemini-1.5-pro-latest",
  safety_settings=safety_settings,
  generation_config=generation_config,
  system_instruction="### Instruction for Traffic Light Intersection Controller\n\n#### Role Overview\nYour role is to configure and optimize the timing of a traffic light intersection. The intersection has the following states:\n\n- **GREEN1_RED2**: Traffic Light 1 is Green, Traffic Light 2 is Red\n- **YELLOW1_RED2**: Traffic Light 1 is Yellow, Traffic Light 2 is Red\n- **RED1_GREEN2**: Traffic Light 1 is Red, Traffic Light 2 is Green\n- **RED1_YELLOW2**: Traffic Light 1 is Red, Traffic Light 2 is Yellow\n- **BLINKING**: Both Traffic Lights are Blinking (e.g., for maintenance or low traffic periods)\n\nEach state has an associated `stateDuration` representing the time (in milliseconds) that the intersection remains in that state.\n\n#### Input\nThe input to your function will be a JSON object containing information about the traffic lights at the intersection. The input structure is as follows:\n```json\n{\n    \"trafficLights\": [\n        {\n            \"trafficLightId\": 1,\n            \"totalOfVehicles\": 0,\n            \"isDaytime\": true,\n            \"co2Level\": 629.9199\n        },\n        {\n            \"trafficLightId\": 2,\n            \"totalOfVehicles\": 0,\n            \"isDaytime\": true,\n            \"co2Level\": 0\n        }\n    ]\n}\n```\n- **trafficLightId**: Unique identifier for the traffic light.\n- **totalOfVehicles**: Number of vehicles currently at the traffic light.\n- **isDaytime**: Boolean indicating if it is daytime.\n- **co2Level**: Current CO2 level at the traffic light location.\n\n#### Output\nYour function should return a JSON object with the following structure:\n```json\n{\n    \"states\": {\n        \"GREEN1_RED2\": 1,\n        \"YELLOW1_RED2\": 1,\n        \"RED1_GREEN2\": 1,\n        \"RED1_YELLOW2\": 1,\n        \"BLINKING\": 1\n    },\n    \"pedestrianReduction\": 1,\n    \"vehicleCount\": {\n        \"diff1\": 1,\n        \"diff2\": 1,\n        \"diff3\": 1\n    }\n}\n```\n- **states**: Dictionary with the state names as keys and their respective durations in milliseconds as values. No value should be 0.\n- **pedestrianReduction**: Time (in milliseconds) reduced from the green state when a pedestrian presses the pedestrian button. No value should be 0.\n- **vehicleCount**: Dictionary indicating the duration multiplier (e.g., 1.2 for 20%) based on the vehicle count difference across three consecutive cycles at Traffic Light 1 (`diff1`), Traffic Light 2 (`diff2`), and the overall intersection (`diff3`). No value should be 0.\n\n#### Optimization Objectives\n1. **Minimize CO2 Levels**: Adjust state durations to reduce CO2 levels.\n2. **Balance Traffic Flow**: Ensure smooth traffic flow by considering the total number of vehicles at each traffic light.\n3. **Daytime/Nighttime Adjustment**: Optimize timings based on whether it is daytime or nighttime.\n4. **Vehicle Count Adjustment**: Adjust green light duration based on the vehicle count multiplier to accommodate traffic flow effectively.\n\nReturns only the expected JSON output with all durations in milliseconds .",
)

chat_session = model.start_chat(
  history=[
  ]
)

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
            response = chat_session.send_message(json.dumps(sensor_data))
            print(response.text)
            save_to_firebase(sensor_data)
            # send_ack()
        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting program.")

finally:
    ser.close()  # Cierra la conexión serial
