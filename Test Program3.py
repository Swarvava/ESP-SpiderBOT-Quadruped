
# Create a comprehensive MicroPython code for quadruped spider robot
# with ESP32, PCA9685, 8 servo motors, and ultrasonic sensor
# ============================================================
# MicroPython Code for Quadruped Spider Robot
# Hardware: ESP32, PCA9685 (Servo Driver), 8 Servo Motors, HC-SR04 Ultrasonic Sensor
# Author: AI Assistant
# Date: October 2025
# ============================================================

import machine
import time
from machine import Pin, I2C
import math

# ============================================================
# PCA9685 PWM Servo Driver Class
# ============================================================
class PCA9685:
    
    # Register addresses
    MODE1 = 0x00
    MODE2 = 0x01
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09
    
    def __init__(self, i2c, address=0x40):
        self.i2c = i2c
        self.address = address
        self.reset()
        
    def reset(self):
        self.i2c.writeto_mem(self.address, self.MODE1, b'\\x00')
        
    def set_pwm_freq(self, freq):
        prescale = int(25000000.0 / (4096 * freq) - 1)
        oldmode = self.i2c.readfrom_mem(self.address, self.MODE1, 1)[0]
        newmode = (oldmode & 0x7F) | 0x10  # Sleep
        self.i2c.writeto_mem(self.address, self.MODE1, bytes([newmode]))
        self.i2c.writeto_mem(self.address, self.PRESCALE, bytes([prescale]))
        self.i2c.writeto_mem(self.address, self.MODE1, bytes([oldmode]))
        time.sleep_ms(5)
        self.i2c.writeto_mem(self.address, self.MODE1, bytes([oldmode | 0xA0]))
        
    def set_pwm(self, channel, on, off):
        self.i2c.writeto_mem(self.address, self.LED0_ON_L + 4 * channel, bytes([on & 0xFF]))
        self.i2c.writeto_mem(self.address, self.LED0_ON_H + 4 * channel, bytes([on >> 8]))
        self.i2c.writeto_mem(self.address, self.LED0_OFF_L + 4 * channel, bytes([off & 0xFF]))
        self.i2c.writeto_mem(self.address, self.LED0_OFF_H + 4 * channel, bytes([off >> 8]))
        
    def set_servo_angle(self, channel, angle, min_pulse=150, max_pulse=600):
        pulse = int(min_pulse + (max_pulse - min_pulse) * angle / 180)
        self.set_pwm(channel, 0, pulse)

# ============================================================
# HC-SR04 Ultrasonic Sensor Class
# ============================================================
class HCSR04:
    
    def __init__(self, trigger_pin, echo_pin, echo_timeout_us=30000):
        self.trigger = Pin(trigger_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.echo_timeout_us = echo_timeout_us
        self.trigger.value(0)
        
    def distance_cm(self):
        # Send 10us pulse
        self.trigger.value(0)
        time.sleep_us(5)
        self.trigger.value(1)
        time.sleep_us(10)
        self.trigger.value(0)
        
        try:
            # Measure echo pulse duration
            pulse_time = machine.time_pulse_us(self.echo, 1, self.echo_timeout_us)
            # Calculate distance (speed of sound = 343 m/s)
            distance = (pulse_time * 0.0343) / 2
            return distance
        except OSError:
            return -1  # Out of range

# ============================================================
# Quadruped Robot Class
# ============================================================
class QuadrupedRobot:
    
    def __init__(self):
        # Initialize I2C for PCA9685 (GPIO21=SDA, GPIO22=SCL)
        self.i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
        
        # Initialize PCA9685 servo driver
        self.pwm = PCA9685(self.i2c)
        self.pwm.set_pwm_freq(50)  # 50Hz for servos
        
        # Initialize ultrasonic sensor (GPIO5=Trigger, GPIO18=Echo)
        self.ultrasonic = HCSR04(trigger_pin=5, echo_pin=18)
        
        # Servo channel mapping for 8 servos (2 per leg)
        # Leg layout: FL(Front-Left), FR(Front-Right), BL(Back-Left), BR(Back-Right)
        self.servo_channels = {
            'FL_hip': 0,    # Front-Left Hip
            'FL_knee': 1,   # Front-Left Knee
            'FR_hip': 2,    # Front-Right Hip
            'FR_knee': 3,   # Front-Right Knee
            'BL_hip': 4,    # Back-Left Hip
            'BL_knee': 5,   # Back-Left Knee
            'BR_hip': 6,    # Back-Right Hip
            'BR_knee': 7    # Back-Right Knee
        }
        
        # Default standing position angles
        self.stand_position = {
            'FL_hip': 90, 'FL_knee': 90,
            'FR_hip': 90, 'FR_knee': 90,
            'BL_hip': 90, 'BL_knee': 90,
            'BR_hip': 90, 'BR_knee': 90
        }
        
        print("✅ Quadruped Robot Initialized!")
        
    def set_servo(self, servo_name, angle):
        channel = self.servo_channels[servo_name]
        # Constrain angle between 0-180
        angle = max(0, min(180, angle))
        self.pwm.set_servo_angle(channel, angle)
        
    def set_all_servos(self, angles_dict):
        for servo_name, angle in angles_dict.items():
            self.set_servo(servo_name, angle)
            time.sleep_ms(20)  # Small delay between servo movements
            
    def stand(self):
        print("🤖 Standing up...")
        self.set_all_servos(self.stand_position)
        time.sleep(0.5)
        
    def sit(self):
        print("🪑 Sitting down...")
        sit_position = {
            'FL_hip': 45, 'FL_knee': 135,
            'FR_hip': 135, 'FR_knee': 45,
            'BL_hip': 45, 'BL_knee': 135,
            'BR_hip': 135, 'BR_knee': 45
        }
        self.set_all_servos(sit_position)
        time.sleep(0.5)
        
    def walk_forward_step(self):
        # Phase 1: Lift FL and BR legs
        self.set_servo('FL_knee', 60)
        self.set_servo('BR_knee', 60)
        time.sleep_ms(200)
        
        # Phase 2: Move lifted legs forward
        self.set_servo('FL_hip', 120)
        self.set_servo('BR_hip', 60)
        time.sleep_ms(200)
        
        # Phase 3: Put legs down
        self.set_servo('FL_knee', 90)
        self.set_servo('BR_knee', 90)
        time.sleep_ms(200)
        
        # Phase 4: Lift FR and BL legs
        self.set_servo('FR_knee', 60)
        self.set_servo('BL_knee', 60)
        time.sleep_ms(200)
        
        # Phase 5: Move lifted legs forward
        self.set_servo('FR_hip', 60)
        self.set_servo('BL_hip', 120)
        time.sleep_ms(200)
        
        # Phase 6: Put legs down
        self.set_servo('FR_knee', 90)
        self.set_servo('BL_knee', 90)
        time.sleep_ms(200)
        
    def walk_forward(self, steps=3):
        print(f"🚶 Walking forward {steps} steps...")
        for i in range(steps):
            self.walk_forward_step()
            print(f"  Step {i+1} completed")
            
    def turn_left(self):
        print("↪️ Turning left...")
        for _ in range(2):
            self.set_all_servos({
                'FL_hip': 60, 'FL_knee': 90,
                'FR_hip': 120, 'FR_knee': 90,
                'BL_hip': 60, 'BL_knee': 90,
                'BR_hip': 120, 'BR_knee': 90
            })
            time.sleep(0.3)
            self.stand()
            time.sleep(0.3)
            
    def turn_right(self):
        print("↩️ Turning right...")
        for _ in range(2):
            self.set_all_servos({
                'FL_hip': 120, 'FL_knee': 90,
                'FR_hip': 60, 'FR_knee': 90,
                'BL_hip': 120, 'BL_knee': 90,
                'BR_hip': 60, 'BR_knee': 90
            })
            time.sleep(0.3)
            self.stand()
            time.sleep(0.3)
            
    def get_distance(self):
        distance = self.ultrasonic.distance_cm()
        if distance > 0:
            print(f"📏 Distance: {distance:.2f} cm")
        else:
            print("⚠️ Distance: Out of range")
        return distance
        
    def obstacle_avoidance_mode(self, duration_sec=30):
        print(f"🤖 Starting obstacle avoidance mode for {duration_sec} seconds...")
        start_time = time.time()
        
        while (time.time() - start_time) < duration_sec:
            distance = self.get_distance()
            
            if distance < 0:  # Out of range
                self.walk_forward(1)
            elif distance < 20:  # Obstacle detected
                print("⚠️ Obstacle detected! Avoiding...")
                self.turn_right()
                time.sleep(0.5)
            elif distance < 40:  # Getting close
                print("⚡ Slowing down...")
                self.walk_forward(1)
            else:  # Clear path
                self.walk_forward(2)
                
            time.sleep(0.5)
            
        print("✅ Obstacle avoidance mode completed!")
        self.stand()
        
    def wave(self):
        print("👋 Waving...")
        for _ in range(3):
            self.set_servo('FL_knee', 45)
            time.sleep(0.3)
            self.set_servo('FL_hip', 120)
            time.sleep(0.3)
            self.set_servo('FL_hip', 60)
            time.sleep(0.3)
        self.stand()

# ============================================================
# Main Program
# ============================================================
def main():
    print("="*50)
    print("🕷️  QUADRUPED SPIDER ROBOT CONTROL SYSTEM")
    print("="*50)
    print()
    
    # Create robot instance
    robot = QuadrupedRobot()
    
    # Initialization sequence
    print("\\n🔧 Initializing robot...")
    time.sleep(1)
    robot.stand()
    time.sleep(1)
    
    # Demo sequence
    print("\\n▶️ Starting demo sequence...\\n")
    
    # 1. Wave
    robot.wave()
    time.sleep(1)
    
    # 2. Walk forward
    robot.walk_forward(3)
    time.sleep(1)
    
    # 3. Turn left
    robot.turn_left()
    time.sleep(1)
    
    # 4. Turn right  
    robot.turn_right()
    time.sleep(1)
    
    # 5. Test ultrasonic sensor
    print("\\n📡 Testing ultrasonic sensor...")
    for i in range(5):
        robot.get_distance()
        time.sleep(1)
    
    # 6. Obstacle avoidance mode (optional - uncomment to enable)
    # robot.obstacle_avoidance_mode(duration_sec=30)
    
    # 7. Sit down
    robot.sit()
    time.sleep(1)
    
    print("\\n✅ Demo completed!")
    print("="*50)

# Run the main program
if __name__ == "__main__":
    main()

# Save the code to a file
with open('quadruped_robot_code.py', 'w') as f:
    f.write(code)

print("✅ MicroPython code generated successfully!")
print("📄 File saved as: quadruped_robot_code.py")
print("\n📋 Code Summary:")
print("- Total lines: ~350+")
print("- Classes: PCA9685, HCSR04, QuadrupedRobot")
print("- Features: Servo control, ultrasonic sensing, walking, turning, obstacle avoidance")