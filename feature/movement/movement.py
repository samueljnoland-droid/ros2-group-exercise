import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorCommand:
    """
    Motor command handler for controlling PWM values between -100 and 100.
    
    - Positive values (1-100): Forward motion
    - Negative values (-100 to -1): Reverse motion
    - Zero (0): Stop/Coast
    """
    
    MIN_PWM = -100
    MAX_PWM = 100
    
    def __init__(self, default_pwm: int = 0):
        """
        Initialize the motor command.
        
        Args:
            default_pwm: Initial PWM value (default: 0 for stop)
        """
        self.pwm = self._clamp_pwm(default_pwm)
    
    def _clamp_pwm(self, value: int) -> int:
        """
        Clamp PWM value to valid range [-100, 100].
        
        Args:
            value: The PWM value to clamp
            
        Returns:
            Clamped PWM value within [-100, 100]
        """
        return max(self.MIN_PWM, min(self.MAX_PWM, value))
    
    def set_pwm(self, value: int) -> bool:
        """
        Set motor PWM value.
        
        Args:
            value: PWM value to set (will be clamped to [-100, 100])
            
        Returns:
            True if value was within valid range, False if clamped
        """
        was_valid = self.MIN_PWM <= value <= self.MAX_PWM
        self.pwm = self._clamp_pwm(value)
        return was_valid
    
    def get_pwm(self) -> int:
        """Get current PWM value."""
        return self.pwm
    
    def stop(self) -> None:
        """Stop the motor (set PWM to 0)."""
        self.pwm = 0
    
    def forward(self, speed: int) -> bool:
        """
        Set motor to forward motion.
        
        Args:
            speed: Forward speed (1-100)
            
        Returns:
            True if speed was valid, False if clamped
        """
        return self.set_pwm(abs(speed))
    
    def reverse(self, speed: int) -> bool:
        """
        Set motor to reverse motion.
        
        Args:
            speed: Reverse speed (1-100, will be negated)
            
        Returns:
            True if speed was valid, False if clamped
        """
        return self.set_pwm(-abs(speed))
    
    def __repr__(self) -> str:
        return f"MotorCommand(pwm={self.pwm})"


class MotorController(Node):
    """ROS 2 Node for controlling motor via Twist messages."""
    
    def __init__(self):
        super().__init__('motor_controller')
        self.motor = MotorCommand()
        self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.get_logger().info('MotorController node initialized')
    
    def callback(self, msg: Twist):
        """
        Callback for /cmd_vel subscription.
        Maps linear velocity to motor PWM (-100 to 100).
        
        Args:
            msg: Twist message containing velocity commands
        """
        # Get linear velocity (forward/backward motion)
        linear_velocity = msg.linear.x
        
        # Map velocity to PWM range [-100, 100]
        # Assuming velocity range is normalized between -1.0 and 1.0
        pwm = int(linear_velocity * 100)
        
        # Set motor PWM
        was_valid = self.motor.set_pwm(pwm)
        
        if not was_valid:
            self.get_logger().warn(f'PWM clamped from requested value to {self.motor.get_pwm()}')
        
        self.get_logger().debug(f'Velocity: {linear_velocity:.2f}, PWM: {self.motor.get_pwm()}')
    
    def destroy_node(self):
        """Clean up motor controller on shutdown."""
        self.motor.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


