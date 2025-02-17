import smbus
import time
import csv
from collections import deque
import math

# MPU6050 registers (keeping your existing definitions)
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# Initialize I2C bus
bus = smbus.SMBus(1)
address = 0x68

# Enhanced configuration
bus.write_byte_data(address, PWR_MGMT_1, 0x00)  # Wake up
bus.write_byte_data(address, SMPLRT_DIV, 0x07)  # Sample rate divider: 1kHz/(1+7) = 125Hz
bus.write_byte_data(address, CONFIG, 0x06)       # DLPF: 5Hz bandwidth for better stability
bus.write_byte_data(address, GYRO_CONFIG, 0x10)  # ±1000 deg/s range
bus.write_byte_data(address, ACCEL_CONFIG, 0x10) # ±8g range

# Sensor filter class
class SensorFilter:
    def __init__(self, window_size=10, threshold=30):
        self.window = deque(maxlen=window_size)
        self.threshold = threshold
        self.last_value = 0
        
    def update(self, new_value):
        # Threshold filter
        if len(self.window) > 0:
            if abs(new_value - self.last_value) > self.threshold:
                new_value = self.last_value
        
        # Moving average filter
        self.window.append(new_value)
        avg_value = sum(self.window) / len(self.window)
        self.last_value = avg_value
        return avg_value

# Complementary filter for sensor fusion
class ComplementaryFilter:
    def __init__(self, alpha=0.96):
        self.alpha = alpha
        self.last_angle = 0
        
    def update(self, gyro_rate, accel_angle, dt):
        angle = self.alpha * (self.last_angle + gyro_rate * dt) + \
                (1 - self.alpha) * accel_angle
        self.last_angle = angle
        return angle

def read_word_2c(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    if value > 32767:
        value -= 65536
    return value

def get_data():
    accel_x = read_word_2c(ACCEL_XOUT_H)
    accel_y = read_word_2c(ACCEL_YOUT_H)
    accel_z = read_word_2c(ACCEL_ZOUT_H)
    gyro_x = read_word_2c(GYRO_XOUT_H)
    gyro_y = read_word_2c(GYRO_YOUT_H)
    gyro_z = read_word_2c(GYRO_ZOUT_H)
    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

# Calibration parameters (keeping your values)
accel_x_bias = 0
accel_y_bias = 0
accel_z_bias = 0
accel_x_scale = 1.0
accel_y_scale = 1.0
accel_z_scale = 1.0

gyro_x_bias = 0
gyro_y_bias = 0
gyro_z_bias = 0
gyro_x_scale = 1.0
gyro_y_scale = 1.0
gyro_z_scale = 1.0

# Initialize filters
gyro_x_filter = SensorFilter(window_size=15, threshold=50)
gyro_y_filter = SensorFilter(window_size=15, threshold=50)
gyro_z_filter = SensorFilter(window_size=15, threshold=50)
accel_x_filter = SensorFilter(window_size=10, threshold=100)
accel_y_filter = SensorFilter(window_size=10, threshold=100)
accel_z_filter = SensorFilter(window_size=10, threshold=100)

# File handling
accel_filename = "accelerometer_data.csv"
gyro_filename = "gyroscope_data.csv"

with open(accel_filename, 'w', newline='') as accel_file, \
     open(gyro_filename, 'w', newline='') as gyro_file:
    
    accel_writer = csv.writer(accel_file)
    gyro_writer = csv.writer(gyro_file)
    
    accel_writer.writerow(["Time (s)", "Accel X", "Accel Y", "Accel Z"])
    gyro_writer.writerow(["Time (s)", "Gyro X", "Gyro Y", "Gyro Z"])
    
    try:
        start_time = time.time()
        last_time = start_time
        end_time = start_time + 30
        
        while time.time() < end_time:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Get raw data
            raw_accel_x, raw_accel_y, raw_accel_z, raw_gyro_x, raw_gyro_y, raw_gyro_z = get_data()
            
            # Apply calibration
            accel_x = (raw_accel_x - accel_x_bias) * accel_x_scale
            accel_y = (raw_accel_y - accel_y_bias) * accel_y_scale
            accel_z = (raw_accel_z - accel_z_bias) * accel_z_scale
            
            gyro_x = (raw_gyro_x - gyro_x_bias) * gyro_x_scale
            gyro_y = (raw_gyro_y - gyro_y_bias) * gyro_y_scale
            gyro_z = (raw_gyro_z - gyro_z_bias) * gyro_z_scale
            
            # Apply filtering
            filtered_gyro_x = gyro_x_filter.update(gyro_x)
            filtered_gyro_y = gyro_y_filter.update(gyro_y)
            filtered_gyro_z = gyro_z_filter.update(gyro_z)
            
            filtered_accel_x = accel_x_filter.update(accel_x)
            filtered_accel_y = accel_y_filter.update(accel_y)
            filtered_accel_z = accel_z_filter.update(accel_z)
            
            elapsed_time = current_time - start_time
            
            # Write filtered data to CSV
            accel_writer.writerow([elapsed_time, filtered_accel_x, filtered_accel_y, filtered_accel_z])
            gyro_writer.writerow([elapsed_time, filtered_gyro_x, filtered_gyro_y, filtered_gyro_z])
            
            print(f"Time: {elapsed_time:.2f}s, Gyro: x={filtered_gyro_x:.2f}, y={filtered_gyro_y:.2f}, z={filtered_gyro_z:.2f}")
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("Exiting...")

print("Data collection complete. Check", accel_filename, "and", gyro_filename)
