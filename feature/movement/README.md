# Motor Control Module

Simple motor control for ROS2 robot

## Usage

```python
from movement import Motor

# Create motor on pin 17
motor = Motor(pin=17, name="MainMotor")

# Run forward at 70% speed
motor.forward(0.7)

# Run backward at 50% speed  
motor.backward(0.5)

# Stop motor
motor.stop()
```

## Requirements

- Python 3.8+
- For Raspberry Pi: `RPi.GPIO` library
- For development/testing on Windows: No additional requirements
