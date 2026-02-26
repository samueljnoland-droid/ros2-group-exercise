#!/usr/bin/env python3
"""
Dual Motor control with PWM using DRV8833 driver on Raspberry Pi 4
Controls Motor A and Motor B via GPIO pins with Pulse Width Modulation
Motor A: A1 = GPIO 19, A2 = GPIO 26
Motor B: B1 = GPIO 20, B2 = GPIO 21
PWM Frequency = 500 Hz
"""

import RPi.GPIO as GPIO
import time

# GPIO Pin Configuration - Motor A
A1 = 19  # Motor A Input 1
A2 = 26  # Motor A Input 2

# GPIO Pin Configuration - Motor B
B1 = 20  # Motor B Input 1
B2 = 21  # Motor B Input 2

# PWM Configuration
PWM_FREQUENCY = 500  # 500 Hz

class MotorController:
    """Control dual motors speed and direction using PWM"""
    
    def __init__(self, pin_a1=A1, pin_a2=A2, pin_b1=B1, pin_b2=B2, frequency=PWM_FREQUENCY):
        """
        Initialize dual motor controller
        Args:
            pin_a1: GPIO pin for motor A input 1
            pin_a2: GPIO pin for motor A input 2
            pin_b1: GPIO pin for motor B input 1
            pin_b2: GPIO pin for motor B input 2
            frequency: PWM frequency in Hz
        """
        self.pin_a1 = pin_a1
        self.pin_a2 = pin_a2
        self.pin_b1 = pin_b1
        self.pin_b2 = pin_b2
        self.frequency = frequency
        
        # PWM objects
        self.pwm_a1 = None
        self.pwm_a2 = None
        self.pwm_b1 = None
        self.pwm_b2 = None
        
        self._setup_gpio()
    
    def _setup_gpio(self):
        """Configure GPIO and initialize PWM for both motors"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Set pins as output
        GPIO.setup(self.pin_a1, GPIO.OUT)
        GPIO.setup(self.pin_a2, GPIO.OUT)
        GPIO.setup(self.pin_b1, GPIO.OUT)
        GPIO.setup(self.pin_b2, GPIO.OUT)
        
        # Initialize PWM for Motor A
        self.pwm_a1 = GPIO.PWM(self.pin_a1, self.frequency)
        self.pwm_a2 = GPIO.PWM(self.pin_a2, self.frequency)
        
        # Initialize PWM for Motor B
        self.pwm_b1 = GPIO.PWM(self.pin_b1, self.frequency)
        self.pwm_b2 = GPIO.PWM(self.pin_b2, self.frequency)
        
        # Start all PWM at 0% duty cycle
        self.pwm_a1.start(0)
        self.pwm_a2.start(0)
        self.pwm_b1.start(0)
        self.pwm_b2.start(0)
        
        print(f"Dual motor controller initialized - PWM Frequency: {self.frequency} Hz")
    
    def forward(self, speed=100, motor='both'):
        """
        Rotate motor(s) forward
        Args:
            speed: Motor speed 0-100 (percentage duty cycle)
            motor: 'A', 'B', or 'both'
        """
        speed = max(0, min(100, speed))  # Clamp speed 0-100
        
        if motor in ['A', 'both']:
            self.pwm_a1.ChangeDutyCycle(speed)
            self.pwm_a2.ChangeDutyCycle(0)
            print(f"Motor A forward - Speed: {speed}%")
        
        if motor in ['B', 'both']:
            self.pwm_b1.ChangeDutyCycle(speed)
            self.pwm_b2.ChangeDutyCycle(0)
            print(f"Motor B forward - Speed: {speed}%")
    
    def backward(self, speed=100, motor='both'):
        """
        Rotate motor(s) backward
        Args:
            speed: Motor speed 0-100 (percentage duty cycle)
            motor: 'A', 'B', or 'both'
        """
        speed = max(0, min(100, speed))  # Clamp speed 0-100
        
        if motor in ['A', 'both']:
            self.pwm_a1.ChangeDutyCycle(0)
            self.pwm_a2.ChangeDutyCycle(speed)
            print(f"Motor A backward - Speed: {speed}%")
        
        if motor in ['B', 'both']:
            self.pwm_b1.ChangeDutyCycle(0)
            self.pwm_b2.ChangeDutyCycle(speed)
            print(f"Motor B backward - Speed: {speed}%")
    
    def stop(self, motor='both'):
        """
        Stop motor(s)
        Args:
            motor: 'A', 'B', or 'both'
        """
        if motor in ['A', 'both']:
            self.pwm_a1.ChangeDutyCycle(0)
            self.pwm_a2.ChangeDutyCycle(0)
            print("Motor A stopped")
        
        if motor in ['B', 'both']:
            self.pwm_b1.ChangeDutyCycle(0)
            self.pwm_b2.ChangeDutyCycle(0)
            print("Motor B stopped")
    
    def cleanup(self):
        """Cleanup GPIO and PWM"""
        self.pwm_a1.stop()
        self.pwm_a2.stop()
        self.pwm_b1.stop()
        self.pwm_b2.stop()
        GPIO.cleanup()
        print("GPIO cleanup complete")


def main():
    """Main loop - test both motors, stop after 3 forward/backward cycles"""
    motor = None
    try:
        motor = MotorController()
        cycle_count = 0
        max_cycles = 3
        
        # Test sequence with both motors
        while cycle_count < max_cycles:
            # Both motors forward for 2 seconds
            print(f"\n--- Cycle {cycle_count + 1}/{max_cycles}: Both Motors Forward ---")
            motor.forward(speed=100, motor='both')
            time.sleep(2)
            
            # Both motors backward for 2 seconds
            print(f"--- Cycle {cycle_count + 1}/{max_cycles}: Both Motors Backward ---")
            motor.backward(speed=100, motor='both')
            time.sleep(2)
            
            cycle_count += 1
        
        # Stop motors after 3 cycles
        print(f"\n--- Completed {max_cycles} cycles. Stopping motors ---")
        motor.stop(motor='both')
        time.sleep(1)
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if motor is not None:
            motor.cleanup()


if __name__ == "__main__":
    main()


if __name__ == "__main__":
    main()