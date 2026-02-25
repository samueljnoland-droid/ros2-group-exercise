import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorController(Node):
    """Motor controller for robot movement"""
    
    def __init__(self):
        super().__init__('motor_controller')
        
        # Create publisher for velocity commands
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Linear speed (m/s) and angular speed (rad/s)
        self.linear_speed = 0.2
        self.angular_speed = 0.0
    
    def move_forward(self, duration=None):
        """
        Move the robot forward
        
        Args:
            duration: How long to move forward (seconds). 
                     If None, publish once and let caller handle timing.
        """
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        
        self.publisher.publish(twist)
        self.get_logger().info(f'Moving forward with speed: {self.linear_speed} m/s')
    
    def stop(self):
        """Stop the robot by setting all velocities to zero"""
        twist = Twist()
        self.publisher.publish(twist)
        self.get_logger().info('Robot stopped')
    
    def set_linear_speed(self, speed):
        """
        Set the linear forward speed
        
        Args:
            speed: Linear speed in m/s
        """
        self.linear_speed = speed
    
    def set_angular_speed(self, speed):
        """
        Set the angular rotation speed
        
        Args:
            speed: Angular speed in rad/s
        """
        self.angular_speed = speed


def main(args=None):
    rclpy.init(args=args)
    
    motor_controller = MotorController()
    
    # Move forward for demonstration
    motor_controller.move_forward()
    
    # Keep the node running
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.stop()
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

