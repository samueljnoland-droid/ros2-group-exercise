"""
Motor control - For Raspberry Pi deployment only
Use movement.py for Windows testing
"""

# This file is deprecated - use movement.py instead
        self.speed = 0
        print("Motor stopped")

try:
    # Run motor at 75% speed for 5 seconds
    pwm.ChangeDutyCycle(75)
    time.sleep(5)
    
    # Stop motor
    pwm.ChangeDutyCycle(0)
    time.sleep(1)
    
    # Run at 50% speed for 3 seconds
    pwm.ChangeDutyCycle(50)
    time.sleep(3)
    
finally:
    # Cleanup
    pwm.stop()
    GPIO.cleanup()

# Usage
motor = Motor(pin=17)
motor.forward(0.7)
time.sleep(2)
motor.backward(0.5)
time.sleep(2)
motor.stop()