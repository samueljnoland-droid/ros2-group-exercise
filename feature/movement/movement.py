"""Motor control module for ROS2 robot"""

import time


class Motor:
    """Simple motor control class"""
    
    def __init__(self, pin: int, name: str = "Motor"):
        """Initialize motor on given pin
        
        Args:
            pin: GPIO pin number
            name: Motor identifier
        """
        self.pin = pin
        self.name = name
        self.speed = 0.0
        print(f"{self.name} initialized on pin {self.pin}")
    
    def forward(self, speed: float) -> None:
        """Run motor forward
        
        Args:
            speed: Speed 0.0 to 1.0
        """
        self.speed = max(0.0, min(1.0, speed))
        print(f"{self.name}: Forward at {self.speed*100:.0f}%")
    
    def backward(self, speed: float) -> None:
        """Run motor backward
        
        Args:
            speed: Speed 0.0 to 1.0
        """
        self.speed = -max(0.0, min(1.0, speed))
        print(f"{self.name}: Backward at {abs(self.speed)*100:.0f}%")
    
    def stop(self) -> None:
        """Stop motor"""
        self.speed = 0.0
        print(f"{self.name}: Stopped")


if __name__ == "__main__":
    # Test motor
    motor = Motor(pin=17, name="MainMotor")
    
    motor.forward(0.7)
    time.sleep(1)
    
    motor.backward(0.5)
    time.sleep(1)
    
    motor.stop()


