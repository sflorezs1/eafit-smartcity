# Arduino to Firebase Data Transfer

<p align="center"><a target="_blank"><img src="./icon.png" width="200"></a></p>

This Python script reads data from an Arduino device via serial communication and sends it to a Firebase Realtime Database.

## Features

- Reads data from Arduino via serial port.
- Sends data to Firebase Realtime Database.
- Automatically attempts to reconnect to the Arduino if the device is disconnected.

## Requirements

- Python 3.x
- `serial` library (can be installed via `pip install pyserial`)
- `firebase-admin` library (can be installed via `pip install firebase-admin`)
- `gemini` library (can be installed via `pip install google-generativeai`)
- Access to Firebase Realtime Database
- Arduino device connected to the computer via USB

## Installation

1. Install Python dependencies:

    ```
    pip install -r requirements.txt
    ```

2. Replace `path/to/serviceAccountKey.json` with your Firebase service account key file.

## Usage

1. Connect your Arduino device to the computer via USB.

2. Modify the `arduino_port` variable in the Python script (`server.py`) to match the serial port of your Arduino device.

3. Run the Python script:

    ```
    python server.py
    ```

4. The script will continuously read data from the Arduino and send it to your Firebase Realtime Database.

## Configuration

- Modify the `arduino_port` variable in the Python script to match the serial port of your Arduino device.
- Replace `path/to/serviceAccountKey.json` with the path to your Firebase service account key file.
