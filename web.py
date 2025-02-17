'''
Read Gyro and Accelerometer and display on a webpage with 3D axes
'''
import smbus
from time import sleep
from flask import Flask, render_template
import threading

# Flask setup
app = Flask(__name__)

# Global variables for sensor data
sensor_data = {
    'Gx': 0.0, 'Gy': 0.0, 'Gz': 0.0,
    'Ax': 0.0, 'Ay': 0.0, 'Az': 0.0
}

# MPU6050 Registers
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

bus = smbus.SMBus(1)
Device_Address = 0x68

def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0x06)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

def sensor_loop():
    MPU_Init()
    while True:
        # Read accelerometer
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)
        # Read gyro
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
        # Update global data
        sensor_data.update({
            'Gx': gyro_x / 131.0,
            'Gy': gyro_y / 131.0,
            'Gz': gyro_z / 131.0,
            'Ax': acc_x / 16384.0,
            'Ay': acc_y / 16384.0,
            'Az': acc_z / 16384.0
        })
        sleep(0.1)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/data')
def data():
    return sensor_data

if __name__ == '__main__':
    # Start sensor thread
    sensor_thread = threading.Thread(target=sensor_loop)
    sensor_thread.daemon = True
    sensor_thread.start()
    # Start Flask app
    app.run(host='0.0.0.0', port=5000, debug=False)
