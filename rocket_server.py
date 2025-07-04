#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Bottle Rocket Telemetry Server (smbus2 version)
# ---------------------------------------------------------------------------
# This script reads sensor data from a BMP280 using the smbus2 library,
# calculates flight metrics, and broadcasts the data over a WebSocket.
#
# To Run:
# 1. Install required libraries: pip install websockets smbus2
# 2. Enable I2C interface using 'sudo raspi-config'
# 3. Run from the terminal: python rocket_server.py
# ---------------------------------------------------------------------------

import asyncio
import json
import time
import datetime
import math

# --- WebSocket Server Libraries ---
import websockets

# --- Sensor Libraries ---
# Using smbus2 for direct I2C communication
try:
    from smbus2 import SMBus
    SMBUS2_ENABLED = True
except ImportError:
    print("WARNING: smbus2 library not found. Run 'pip install smbus2'.")
    SMBUS2_ENABLED = False

# It's assumed the camera stream is handled by a separate process (e.g., mjpeg-streamer)
CAMERA_ENABLED = True 
SERVO_ENABLED = True # Assuming servo can be controlled

# --- Configuration ---
HOST = '0.0.0.0'  # Listen on all network interfaces
PORT = 8765       # WebSocket port the dashboard connects to
DATA_RATE_HZ = 100 # Target data rate
SEA_LEVEL_PRESSURE_HPA = 1013.25 # Standard sea level pressure

# --- Global State Variables ---
clients = set()
apogee = 0.0
last_altitude = 0.0
last_timestamp = time.monotonic()
bmp_sensor = None
status = {
    "bmp280": False,
    "camera": False,
    "servo": False
}

# --- Custom BMP280 Driver using smbus2 ---
class BMP280:
    """
    Custom driver for BMP280 sensor using smbus2.
    Reads calibration data and provides compensated temperature, pressure, and altitude.
    """
    def __init__(self, i2c_bus=1, address=0x76):
        self.bus = SMBus(i2c_bus)
        self.address = address
        self.sea_level_pressure = SEA_LEVEL_PRESSURE_HPA
        self._load_calibration_data()

        # Set sensor mode and oversampling
        # osrs_t: temp oversampling, osrs_p: pressure oversampling, mode: Normal
        # Refer to BMP280 datasheet for these values
        config = (0b101 << 5) | (0b101 << 2) | 0b11
        self.bus.write_byte_data(self.address, 0xF5, 0b10100000) # standby time 1000ms
        self.bus.write_byte_data(self.address, 0xF4, config)
        self.t_fine = 0

    def _load_calibration_data(self):
        # Reads the factory calibration data from the sensor's non-volatile memory
        cal_data = self.bus.read_i2c_block_data(self.address, 0x88, 24)
        self.dig_T1 = self._get_short(cal_data, 0)
        self.dig_T2 = self._get_short(cal_data, 2, signed=True)
        self.dig_T3 = self._get_short(cal_data, 4, signed=True)
        self.dig_P1 = self._get_short(cal_data, 6)
        self.dig_P2 = self._get_short(cal_data, 8, signed=True)
        self.dig_P3 = self._get_short(cal_data, 10, signed=True)
        self.dig_P4 = self._get_short(cal_data, 12, signed=True)
        self.dig_P5 = self._get_short(cal_data, 14, signed=True)
        self.dig_P6 = self._get_short(cal_data, 16, signed=True)
        self.dig_P7 = self._get_short(cal_data, 18, signed=True)
        self.dig_P8 = self._get_short(cal_data, 20, signed=True)
        self.dig_P9 = self._get_short(cal_data, 22, signed=True)

    def _get_short(self, data, index, signed=False):
        # Helper to convert two bytes to a 16-bit integer
        val = (data[index+1] << 8) | data[index]
        if signed and val > 32767:
            val -= 65536
        return val

    def _compensate_T(self, adc_T):
        # Temperature compensation formula from datasheet
        v1 = (adc_T / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        v2 = (adc_T / 131072.0 - self.dig_T1 / 8192.0) * (adc_T / 131072.0 - self.dig_T1 / 8192.0) * self.dig_T3
        self.t_fine = v1 + v2
        return self.t_fine / 5120.0

    def _compensate_P(self, adc_P):
        # Pressure compensation formula from datasheet
        v1 = (self.t_fine / 2.0) - 64000.0
        v2 = v1 * v1 * self.dig_P6 / 32768.0
        v2 = v2 + v1 * self.dig_P5 * 2.0
        v2 = (v2 / 4.0) + (self.dig_P4 * 65536.0)
        v1 = (self.dig_P3 * v1 * v1 / 524288.0 + self.dig_P2 * v1) / 524288.0
        v1 = (1.0 + v1 / 32768.0) * self.dig_P1
        if v1 == 0: return 0
        p = 1048576.0 - adc_P
        p = (p - (v2 / 4096.0)) * 6250.0 / v1
        v1 = self.dig_P9 * p * p / 2147483648.0
        v2 = p * self.dig_P8 / 32768.0
        p = p + (v1 + v2 + self.dig_P7) / 16.0
        return p / 100.0 # Return pressure in hPa

    @property
    def pressure(self):
        data = self.bus.read_i2c_block_data(self.address, 0xF7, 6)
        adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        self._compensate_T(adc_T) # Must be called to update t_fine
        return self._compensate_P(adc_P)

    @property
    def altitude(self):
        # Altitude calculation based on pressure
        p = self.pressure
        return 44330.0 * (1.0 - math.pow(p / self.sea_level_pressure, 1.0/5.255))

# --- Sensor Initialization ---
def initialize_sensors():
    global bmp_sensor, status
    if SMBUS2_ENABLED:
        try:
            # Default I2C bus is 1 on most Raspberry Pi models
            # Default BMP280 address is 0x76 or 0x77
            bmp_sensor = BMP280(i2c_bus=1, address=0x76)
            status["bmp280"] = True
            print("INFO: BMP280 sensor initialized successfully using smbus2.")
        except Exception as e:
            print(f"ERROR: Could not initialize BMP280 via smbus2. Check connection and I2C address. {e}")
            status["bmp280"] = False
    
    if CAMERA_ENABLED: status["camera"] = True
    if SERVO_ENABLED: status["servo"] = True

# --- Data Acquisition and Calculation ---
def get_telemetry_data():
    global apogee, last_altitude, last_timestamp
    current_time = time.monotonic()
    time_delta = current_time - last_timestamp
    
    if status["bmp280"] and bmp_sensor:
        altitude = bmp_sensor.altitude
        pressure = bmp_sensor.pressure
    else:
        altitude, pressure = 0.0, 0.0

    velocity = (altitude - last_altitude) / time_delta if time_delta > 0 else 0.0
    if altitude > apogee: apogee = altitude
        
    last_altitude, last_timestamp = altitude, current_time

    js_timestamp = datetime.datetime.utcnow().isoformat() + "Z"
    data = {
        "timestamp": js_timestamp, "altitude": altitude, "apogee": apogee,
        "velocity": velocity, "pressure": pressure, "status": status,
        "camera_url": f"http://{HOST}:8080/?action=stream" if status["camera"] else ""
    }
    return data

# --- WebSocket Server Logic (Unchanged) ---
async def handler(websocket, path):
    global clients
    clients.add(websocket)
    print(f"INFO: Client connected from {websocket.remote_address}")
    try:
        await websocket.wait_closed()
    finally:
        clients.remove(websocket)
        print(f"INFO: Client disconnected from {websocket.remote_address}")

async def broadcast_data():
    while True:
        if clients:
            json_data = json.dumps(get_telemetry_data())
            await asyncio.gather(*[client.send(json_data) for client in clients])
        await asyncio.sleep(1.0 / DATA_RATE_HZ)

# --- Main Execution ---
async def main():
    initialize_sensors()
    server = await websockets.serve(handler, HOST, PORT)
    print(f"INFO: WebSocket server started on ws://{HOST}:{PORT}")
    await broadcast_data()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nINFO: Server shutting down.")
