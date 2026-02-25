#!/usr/bin/env python3
import rclpy
from feature.movement.movement import MotorController


def main():
    rclpy.init()
    
    # Create motor controller
    motor_controller = MotorController()
    
    # Example: Move forward
    motor_controller.move_forward()
    
    # Keep running
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.stop()
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
