# This code is to read the IMU values
# The IMU im using is a 10 axis sensor with on board low power ICM20948
# I will be using the gyroscope values 
#The pins are as follow: 
# SDA -> GPIO2
# SCL -> GPIO3

import board
import busio
import time
from adafruit_icm20x import ICM20948

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize ICM20948 sensor
sensor = ICM20948(i2c)

print("IMU Sensor Initialized")
print("Reading Gyroscope Values...\n")

try:
    while True:
        # Read gyroscope values (in rad/s)
        gyro_x, gyro_y, gyro_z = sensor.gyro
        
        # Read accelerometer values (optional, for reference)
        accel_x, accel_y, accel_z = sensor.acceleration
        
        # Read magnetometer values (optional, for reference)
        mag_x, mag_y, mag_z = sensor.magnetic
        
        print(f"Gyroscope (rad/s):")
        print(f"  X: {gyro_x:.2f}")
        print(f"  Y: {gyro_y:.2f}")
        print(f"  Z: {gyro_z:.2f}")
        
        print(f"\nAccelerometer (m/s^2):")
        print(f"  X: {accel_x:.2f}")
        print(f"  Y: {accel_y:.2f}")
        print(f"  Z: {accel_z:.2f}")
        
        print(f"\nMagnetometer (uT):")
        print(f"  X: {mag_x:.2f}")
        print(f"  Y: {mag_y:.2f}")
        print(f"  Z: {mag_z:.2f}")
        print("-" * 40)
        
        time.sleep(0.5)  # Update every 500ms

except KeyboardInterrupt:
    print("\nSensor reading stopped.")
    i2c.deinit()