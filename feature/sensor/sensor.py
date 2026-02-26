"""IMU sensor module for ROS2 robot"""

import time
from typing import Dict


class IMUSensor:
    """MPU6050 IMU sensor class"""
    
    def __init__(self, address: int = 0x68, name: str = "IMU"):
        """Initialize IMU sensor
        
        Args:
            address: I2C address (default 0x68 for MPU6050)
            name: Sensor identifier
        """
        self.address = address
        self.name = name
        
        try:
            from mpu6050 import mpu6050
            self.imu = mpu6050(address)
            print(f"{self.name} initialized successfully at 0x{address:02x}")
            self.connected = True
        except ImportError:
            print(f"Warning: mpu6050 not installed. Using simulation mode.")
            self.imu = None
            self.connected = False
    
    def read_acceleration(self) -> Dict[str, float]:
        """Read acceleration values (X, Y, Z) in m/s²"""
        if self.connected and self.imu:
            accel = self.imu.get_accel_data()
            return {
                "x": accel['x'],
                "y": accel['y'],
                "z": accel['z']
            }
        else:
            # Simulated values
            return {"x": 0.15, "y": -0.05, "z": 9.81}
    
    def read_gyroscope(self) -> Dict[str, float]:
        """Read gyroscope values (X, Y, Z) in deg/s"""
        if self.connected and self.imu:
            gyro = self.imu.get_gyro_data()
            return {
                "x": gyro['x'],
                "y": gyro['y'],
                "z": gyro['z']
            }
        else:
            # Simulated values
            return {"x": 2.5, "y": -1.2, "z": 0.8}
    
    def read_temperature(self) -> float:
        """Read temperature sensor in °C"""
        if self.connected and self.imu:
            return self.imu.get_temp()
        else:
            # Simulated value
            return 25.5
    
    def read_all(self) -> Dict:
        """Read all sensor values"""
        return {
            "acceleration": self.read_acceleration(),
            "gyroscope": self.read_gyroscope(),
            "temperature": self.read_temperature()
        }
    
    def print_values(self) -> None:
        """Print formatted sensor values"""
        accel = self.read_acceleration()
        gyro = self.read_gyroscope()
        temp = self.read_temperature()
        
        print(f"\n--- {self.name} ---")
        print(f"Acceleration: X={accel['x']:7.2f} Y={accel['y']:7.2f} Z={accel['z']:7.2f} m/s²")
        print(f"Gyroscope:    X={gyro['x']:7.2f} Y={gyro['y']:7.2f} Z={gyro['z']:7.2f} deg/s")
        print(f"Temperature:  {temp:6.1f}°C")


if __name__ == "__main__":
    # Initialize sensor
    imu = IMUSensor(address=0x68, name="MPU6050")
    
    # Read values continuously
    try:
        for i in range(10):
            imu.print_values()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nSensor reading stopped")

