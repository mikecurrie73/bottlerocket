#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Integrated Bottle Rocket Telemetry Server (with Altitude Zeroing)
# ---------------------------------------------------------------------------
# This version adds the ability to receive a command from the dashboard
# to zero the altitude at the current pressure level.
#
# To Run:
# 1. Place index.html in the same directory as this script.
# 2. Run the `start.sh` script from your terminal.
# ---------------------------------------------------------------------------

import asyncio
import json
import time
import datetime
import math
import threading
import http.server
import socketserver
import os

# --- WebSocket Server Libraries ---
import websockets

# --- Sensor Libraries ---
try:
    from smbus2 import SMBus
    SMBUS2_ENABLED = True
except ImportError:
    print("WARNING: smbus2 library not found. Run 'pip install smbus2'.")
    SMBUS2_ENABLED = False

# These flags are now just for the status report. The actual services are run externally.
CAMERA_ENABLED = True 
SERVO_ENABLED = True

# --- Configuration ---
HOST = '0.0.0.0'
WEBSOCKET_PORT = 8765
HTTP_PORT = 8000
DATA_RATE_HZ = 100

# --- Global State Variables ---
clients = set()
apogee = 0.0
last_altitude = 0.0
last_timestamp = time.monotonic()
bmp_sensor = None
ground_level_pressure = None # Will store the pressure when altitude is zeroed
status = {
    "bmp280": False,
    "camera": True,
    "servo": False
}

# --- Custom BMP280 Driver (Omitted for brevity, no changes) ---
class BMP280:
    def __init__(self, i2c_bus=1, address=0x76):
        self.bus = SMBus(i2c_bus)
        self.address = address
        self._load_calibration_data()
        config = (0b101 << 5) | (0b101 << 2) | 0b11
        self.bus.write_byte_data(self.address, 0xF5, 0b10100000)
        self.bus.write_byte_data(self.address, 0xF4, config)
        self.t_fine = 0
    def _load_calibration_data(self):
        cal_data = self.bus.read_i2c_block_data(self.address, 0x88, 24)
        self.dig_T1 = self._get_short(cal_data, 0); self.dig_T2 = self._get_short(cal_data, 2, signed=True); self.dig_T3 = self._get_short(cal_data, 4, signed=True); self.dig_P1 = self._get_short(cal_data, 6); self.dig_P2 = self._get_short(cal_data, 8, signed=True); self.dig_P3 = self._get_short(cal_data, 10, signed=True); self.dig_P4 = self._get_short(cal_data, 12, signed=True); self.dig_P5 = self._get_short(cal_data, 14, signed=True); self.dig_P6 = self._get_short(cal_data, 16, signed=True); self.dig_P7 = self._get_short(cal_data, 18, signed=True); self.dig_P8 = self._get_short(cal_data, 20, signed=True); self.dig_P9 = self._get_short(cal_data, 22, signed=True)
    def _get_short(self, data, index, signed=False):
        val = (data[index+1] << 8) | data[index]
        if signed and val > 32767: val -= 65536
        return val
    def _compensate_T(self, adc_T):
        v1 = (adc_T / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2; v2 = (adc_T / 131072.0 - self.dig_T1 / 8192.0) * (adc_T / 131072.0 - self.dig_T1 / 8192.0) * self.dig_T3; self.t_fine = v1 + v2; return self.t_fine / 5120.0
    def _compensate_P(self, adc_P):
        v1 = (self.t_fine / 2.0) - 64000.0; v2 = v1 * v1 * self.dig_P6 / 32768.0; v2 = v2 + v1 * self.dig_P5 * 2.0; v2 = (v2 / 4.0) + (self.dig_P4 * 65536.0); v1 = (self.dig_P3 * v1 * v1 / 524288.0 + self.dig_P2 * v1) / 524288.0; v1 = (1.0 + v1 / 32768.0) * self.dig_P1;
        if v1 == 0: return 0
        p = 1048576.0 - adc_P; p = (p - (v2 / 4096.0)) * 6250.0 / v1; v1 = self.dig_P9 * p * p / 2147483648.0; v2 = p * self.dig_P8 / 32768.0; p = p + (v1 + v2 + self.dig_P7) / 16.0; return p / 100.0
    @property
    def pressure(self):
        data = self.bus.read_i2c_block_data(self.address, 0xF7, 6); adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4); adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4); self._compensate_T(adc_T); return self._compensate_P(adc_P)
    
    # The altitude calculation is now separate from the class
    def get_altitude(self, pressure, sea_level_pressure):
        return 44330.0 * (1.0 - math.pow(pressure / sea_level_pressure, 1.0/5.255))

# --- Sensor Initialization ---
def initialize_sensors():
    global bmp_sensor, status, ground_level_pressure
    if SMBUS2_ENABLED:
        try:
            bmp_sensor = BMP280(i2c_bus=1, address=0x76)
            # Set initial ground level pressure on startup
            ground_level_pressure = bmp_sensor.pressure
            status["bmp280"] = True
            print(f"INFO: BMP280 sensor initialized. Initial ground pressure: {ground_level_pressure:.2f} hPa")
        except Exception as e:
            print(f"ERROR: Could not initialize BMP280. {e}")
            status["bmp280"] = False
    
    if not CAMERA_ENABLED: status["camera"] = False
    if SERVO_ENABLED: status["servo"] = True

# --- Data Acquisition and Calculation ---
def get_telemetry_data():
    global apogee, last_altitude, last_timestamp
    current_time = time.monotonic(); time_delta = current_time - last_timestamp
    
    if status["bmp280"] and bmp_sensor:
        pressure = bmp_sensor.pressure
        # Calculate altitude based on the current ground_level_pressure
        altitude = bmp_sensor.get_altitude(pressure, ground_level_pressure)
    else: 
        altitude, pressure = 0.0, 0.0

    velocity = (altitude - last_altitude) / time_delta if time_delta > 0 else 0.0
    if altitude > apogee: apogee = altitude
    last_altitude, last_timestamp = altitude, current_time
    js_timestamp = datetime.datetime.utcnow().isoformat() + "Z"
    return {"timestamp": js_timestamp, "altitude": altitude, "apogee": apogee, "velocity": velocity, "pressure": pressure, "status": status}

# --- WebSocket Server Logic ---
async def handler(websocket):
    global ground_level_pressure, apogee, last_altitude
    clients.add(websocket)
    print(f"INFO: Client connected from {websocket.remote_address}")
    try:
        # Listen for messages from the client
        async for message in websocket:
            try:
                data = json.loads(message)
                if data.get("command") == "zero_altitude":
                    if bmp_sensor:
                        ground_level_pressure = bmp_sensor.pressure
                        apogee = 0.0
                        last_altitude = 0.0
                        print(f"ACTION: Altitude zeroed. New ground pressure: {ground_level_pressure:.2f} hPa")
            except json.JSONDecodeError:
                print(f"WARNING: Received invalid JSON message: {message}")

    finally:
        clients.remove(websocket)
        print(f"INFO: Client disconnected from {websocket.remote_address}")

async def broadcast_data():
    while True:
        try:
            if clients:
                telemetry_data = get_telemetry_data()
                json_data = json.dumps(telemetry_data)
                await asyncio.gather(*[client.send(json_data) for client in clients])
            await asyncio.sleep(1.0 / DATA_RATE_HZ)
        except Exception as e:
            print(f"ERROR in broadcast loop: {e}"); await asyncio.sleep(1)

# --- HTTP Server Logic ---
def start_http_server():
    Handler = http.server.SimpleHTTPRequestHandler
    httpd = socketserver.TCPServer((HOST, HTTP_PORT), Handler)
    print(f"INFO: HTTP server started. Open http://<YOUR_PI_IP>:{HTTP_PORT} in a browser.")
    httpd.serve_forever()

# --- Main Execution ---
async def main():
    initialize_sensors()
    
    http_thread = threading.Thread(target=start_http_server, daemon=True)
    http_thread.start()
    
    server = await websockets.serve(handler, HOST, WEBSOCKET_PORT)
    print(f"INFO: WebSocket server started on ws://{HOST}:{WEBSOCKET_PORT}")
    
    await broadcast_data()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nINFO: Server shutting down.")

