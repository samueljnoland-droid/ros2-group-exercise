"""Real-time IMU value viewer for Raspberry Pi"""

import time
import sys
from sensor import IMUSensor


def clear_screen():
    """Clear terminal screen"""
    os.system('clear' if sys.platform != 'win32' else 'cls')


def display_header():
    """Display viewer header"""
    print("=" * 70)
    print("  IMU SENSOR VALUE VIEWER - MPU6050".center(70))
    print("=" * 70)


def display_values(imu: IMUSensor, iteration: int):
    """Display formatted IMU values"""
    accel = imu.read_acceleration()
    gyro = imu.read_gyroscope()
    temp = imu.read_temperature()
    
    print(f"\nReading #{iteration}")
    print("-" * 70)
    
    print(f"\n📊 ACCELERATION (m/s²):")
    print(f"   X-axis: {accel['x']:8.3f} m/s²")
    print(f"   Y-axis: {accel['y']:8.3f} m/s²")
    print(f"   Z-axis: {accel['z']:8.3f} m/s²")
    
    print(f"\n🔄 GYROSCOPE (deg/s):")
    print(f"   X-axis: {gyro['x']:8.3f} deg/s")
    print(f"   Y-axis: {gyro['y']:8.3f} deg/s")
    print(f"   Z-axis: {gyro['z']:8.3f} deg/s")
    
    print(f"\n🌡️  TEMPERATURE:")
    print(f"   Value:  {temp:8.2f}°C")
    
    print("\n" + "-" * 70)


def main():
    """Main viewer loop"""
    try:
        # Initialize IMU
        imu = IMUSensor(address=0x68, name="MPU6050")
        
        display_header()
        print(f"\n✓ Sensor initialized and ready")
        print(f"✓ Reading values every 1 second")
        print(f"✓ Press Ctrl+C to stop\n")
        
        iteration = 0
        while True:
            iteration += 1
            display_values(imu, iteration)
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\n\n" + "=" * 70)
        print("  VIEWER STOPPED".center(70))
        print("=" * 70)
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    import os
    main()
