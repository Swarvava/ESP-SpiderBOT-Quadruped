"""
Quadruped Spider Robot Controller
Hardware:
- ESP32 Microcontroller
- PCA9685 Servo Driver (16-channel)
- 8 Servo Motors (2 per leg: hip and knee)
- HC-SR04 Ultrasonic Sensor
"""

from machine import Pin, I2C
import time
import math

# ========== PCA9685 Servo Driver Class ==========
class PCA9685:
    """Driver for PCA9685 16-channel PWM servo controller"""
    
    def __init__(self, i2c, address=0x40):
        # i2c: I2C object for communication
        # address: I2C address of PCA9685 (default 0x40)
        self.i2c = i2c
        self.address = address
        self.reset()  # Reset the board on initialization
        
    def reset(self):
        """Resets the PCA9685 board"""
        # Write 0x00 to MODE1 register (0x00) to reset
        self.i2c.writeto_mem(self.address, 0x00, b'\x00')
        
    def set_pwm_freq(self, freq_hz):
        """Sets the PWM frequency for all channels"""
        # freq_hz: desired frequency in Hz (typically 50Hz for servos)
        prescaleval = 25000000.0    # 25MHz internal oscillator
        prescaleval /= 4096.0       # 12-bit resolution
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        prescale = int(prescaleval + 0.5)  # Round to nearest integer
        
        # Read current MODE1 register
        oldmode = self.i2c.readfrom_mem(self.address, 0x00, 1)[0]
        # Enter sleep mode to change prescaler
        newmode = (oldmode & 0x7F) | 0x10
        self.i2c.writeto_mem(self.address, 0x00, bytes([newmode]))
        # Write prescaler value
        self.i2c.writeto_mem(self.address, 0xFE, bytes([prescale]))
        # Wake up and enable auto-increment
        self.i2c.writeto_mem(self.address, 0x00, bytes([oldmode]))
        time.sleep_ms(5)
        self.i2c.writeto_mem(self.address, 0x00, bytes([oldmode | 0xA0]))
        
    def set_pwm(self, channel, on, off):
        """Sets PWM values for a specific channel"""
        # channel: servo channel (0-15)
        # on: when to turn on (0-4095)
        # off: when to turn off (0-4095)
        base_reg = 0x06 + 4 * channel  # Calculate register address
        # Write ON_L, ON_H, OFF_L, OFF_H registers
        self.i2c.writeto_mem(self.address, base_reg, bytes([on & 0xFF]))
        self.i2c.writeto_mem(self.address, base_reg + 1, bytes([on >> 8]))
        self.i2c.writeto_mem(self.address, base_reg + 2, bytes([off & 0xFF]))
        self.i2c.writeto_mem(self.address, base_reg + 3, bytes([off >> 8]))


# ========== Ultrasonic Sensor Class ==========
class HCSR04:
    """Driver for HC-SR04 ultrasonic distance sensor"""
    
    def __init__(self, trigger_pin, echo_pin):
        # trigger_pin: GPIO pin number for trigger
        # echo_pin: GPIO pin number for echo
        self.trigger = Pin(trigger_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.trigger.value(0)  # Initialize trigger to LOW
        
    def distance_cm(self):
        """Measures distance in centimeters"""
        # Send 10us trigger pulse
        self.trigger.value(0)
        time.sleep_us(2)
        self.trigger.value(1)
        time.sleep_us(10)
        self.trigger.value(0)
        
        # Wait for echo pin to go HIGH (start of echo)
        timeout = 30000  # 30ms timeout
        start = time.ticks_us()
        while self.echo.value() == 0:
            if time.ticks_diff(time.ticks_us(), start) > timeout:
                return -1  # Timeout, no echo received
            pulse_start = time.ticks_us()
            
        # Wait for echo pin to go LOW (end of echo)
        while self.echo.value() == 1:
            if time.ticks_diff(time.ticks_us(), start) > timeout:
                return -1  # Timeout
            pulse_end = time.ticks_us()
            
        # Calculate distance: (time * speed of sound) / 2
        # Speed of sound = 343 m/s = 0.0343 cm/us
        pulse_duration = time.ticks_diff(pulse_end, pulse_start)
        distance = (pulse_duration * 0.0343) / 2
        return distance


# ========== Servo Class ==========
class Servo:
    """Controls individual servo motors through PCA9685"""
    
    def __init__(self, pca, channel, min_pulse=500, max_pulse=2500):
        # pca: PCA9685 object
        # channel: servo channel on PCA9685 (0-15)
        # min_pulse: minimum pulse width in microseconds (0 degrees)
        # max_pulse: maximum pulse width in microseconds (180 degrees)
        self.pca = pca
        self.channel = channel
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        
    def angle(self, degrees):
        """Sets servo to specific angle (0-180 degrees)"""
        # Constrain angle between 0 and 180
        degrees = max(0, min(180, degrees))
        # Map angle to pulse width
        pulse_width = self.min_pulse + (degrees / 180.0) * (self.max_pulse - self.min_pulse)
        # Convert pulse width to 12-bit PWM value
        # At 50Hz: each tick = 1000000us / 50Hz / 4096 = 4.88us
        pulse_ticks = int(pulse_width / 4.88)
        # Set PWM (ON at 0, OFF at calculated ticks)
        self.pca.set_pwm(self.channel, 0, pulse_ticks)


# ========== Quadruped Robot Class ==========
class QuadrupedRobot:
    """Main robot controller with walking and obstacle avoidance"""
    
    def __init__(self):
        # Initialize I2C for servo driver (SDA=GPIO21, SCL=GPIO22)
        i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
        
        # Initialize PCA9685 servo driver
        self.pca = PCA9685(i2c)
        self.pca.set_pwm_freq(50)  # Set frequency to 50Hz (standard for servos)
        
        # Initialize ultrasonic sensor (TRIG=GPIO5, ECHO=GPIO18)
        self.ultrasonic = HCSR04(trigger_pin=5, echo_pin=18)
        
        # Define servo channels for each leg
        # Each leg has 2 servos: hip (lifts leg) and knee (extends leg)
        # Leg naming: FL=Front Left, FR=Front Right, BL=Back Left, BR=Back Right
        
        # Front Left Leg
        self.fl_hip = Servo(self.pca, 0)    # Channel 0: lifts/lowers leg
        self.fl_knee = Servo(self.pca, 1)   # Channel 1: extends/retracts leg
        
        # Front Right Leg
        self.fr_hip = Servo(self.pca, 2)    # Channel 2
        self.fr_knee = Servo(self.pca, 3)   # Channel 3
        
        # Back Left Leg
        self.bl_hip = Servo(self.pca, 4)    # Channel 4
        self.bl_knee = Servo(self.pca, 5)   # Channel 5
        
        # Back Right Leg
        self.br_hip = Servo(self.pca, 6)    # Channel 6
        self.br_knee = Servo(self.pca, 7)   # Channel 7
        
        # Define servo angle positions
        # Hip: 90 = neutral, <90 = leg down, >90 = leg up
        # Knee: 90 = neutral, <90 = retracted, >90 = extended
        self.hip_up = 120      # Lift leg up
        self.hip_down = 60     # Put leg down
        self.knee_forward = 120  # Extend leg forward
        self.knee_back = 60      # Pull leg back
        self.neutral = 90        # Neutral position
        
        # Safety distance for obstacle avoidance (in cm)
        self.safe_distance = 20
        
        # Move all servos to neutral position at startup
        self.stand_neutral()
        
    def stand_neutral(self):
        """Moves all servos to neutral standing position"""
        print("Moving to neutral position...")
        # Set all hips and knees to 90 degrees (neutral)
        self.fl_hip.angle(self.neutral)
        self.fl_knee.angle(self.neutral)
        self.fr_hip.angle(self.neutral)
        self.fr_knee.angle(self.neutral)
        self.bl_hip.angle(self.neutral)
        self.bl_knee.angle(self.neutral)
        self.br_hip.angle(self.neutral)
        self.br_knee.angle(self.neutral)
        time.sleep(1)  # Wait for servos to reach position
        
    def check_obstacle(self):
        """Checks for obstacles using ultrasonic sensor"""
        distance = self.ultrasonic.distance_cm()
        # Return True if obstacle is too close or sensor error
        if distance == -1 or distance < self.safe_distance:
            return True
        return False
        
    def walk_forward(self, steps=1):
        """Walks forward using alternating tripod gait"""
        # Tripod gait: 3 legs move together while other 3 support
        # Group 1: FL, BR, BL (Front-Left, Back-Right, Back-Left)
        # Group 2: FR, BL, BR (Front-Right, Back-Left, Back-Right)
        
        print(f"Walking forward {steps} steps...")
        
        for step in range(steps):
            # Check for obstacles before each step
            if self.check_obstacle():
                print("Obstacle detected! Stopping.")
                return False
                
            # === Phase 1: Lift and move Group 1 (FL, BR, BL) ===
            # Lift legs
            self.fl_hip.angle(self.hip_up)
            self.br_hip.angle(self.hip_up)
            time.sleep(0.2)
            
            # Move legs forward
            self.fl_knee.angle(self.knee_forward)
            self.br_knee.angle(self.knee_forward)
            time.sleep(0.2)
            
            # Put legs down
            self.fl_hip.angle(self.hip_down)
            self.br_hip.angle(self.hip_down)
            time.sleep(0.2)
            
            # === Phase 2: Move Group 2 (FR, BL) backward (push body forward) ===
            self.fr_knee.angle(self.knee_back)
            self.bl_knee.angle(self.knee_back)
            time.sleep(0.2)
            
            # === Phase 3: Lift and move Group 2 (FR, BL) ===
            # Lift legs
            self.fr_hip.angle(self.hip_up)
            self.bl_hip.angle(self.hip_up)
            time.sleep(0.2)
            
            # Move legs forward
            self.fr_knee.angle(self.knee_forward)
            self.bl_knee.angle(self.knee_forward)
            time.sleep(0.2)
            
            # Put legs down
            self.fr_hip.angle(self.hip_down)
            self.bl_hip.angle(self.hip_down)
            time.sleep(0.2)
            
            # === Phase 4: Move Group 1 backward (push body forward) ===
            self.fl_knee.angle(self.knee_back)
            self.br_knee.angle(self.knee_back)
            time.sleep(0.2)
            
            # Return to neutral position
            self.stand_neutral()
            time.sleep(0.1)
            
        return True
        
    def turn_right(self, angle_steps=1):
        """Turns robot to the right"""
        print(f"Turning right...")
        
        for _ in range(angle_steps):
            # Lift left side legs
            self.fl_hip.angle(self.hip_up)
            self.bl_hip.angle(self.hip_up)
            time.sleep(0.2)
            
            # Move left legs forward (clockwise rotation)
            self.fl_knee.angle(self.knee_forward)
            self.bl_knee.angle(self.knee_forward)
            time.sleep(0.2)
            
            # Put left legs down
            self.fl_hip.angle(self.hip_down)
            self.bl_hip.angle(self.hip_down)
            time.sleep(0.2)
            
            # Lift right side legs
            self.fr_hip.angle(self.hip_up)
            self.br_hip.angle(self.hip_up)
            time.sleep(0.2)
            
            # Move right legs backward (clockwise rotation)
            self.fr_knee.angle(self.knee_back)
            self.br_knee.angle(self.knee_back)
            time.sleep(0.2)
            
            # Put right legs down
            self.fr_hip.angle(self.hip_down)
            self.br_hip.angle(self.hip_down)
            time.sleep(0.2)
            
            # Return to neutral
            self.stand_neutral()
            time.sleep(0.1)
            
    def turn_left(self, angle_steps=1):
        """Turns robot to the left"""
        print(f"Turning left...")
        
        for _ in range(angle_steps):
            # Lift right side legs
            self.fr_hip.angle(self.hip_up)
            self.br_hip.angle(self.hip_up)
            time.sleep(0.2)
            
            # Move right legs forward (counter-clockwise rotation)
            self.fr_knee.angle(self.knee_forward)
            self.br_knee.angle(self.knee_forward)
            time.sleep(0.2)
            
            # Put right legs down
            self.fr_hip.angle(self.hip_down)
            self.br_hip.angle(self.hip_down)
            time.sleep(0.2)
            
            # Lift left side legs
            self.fl_hip.angle(self.hip_up)
            self.bl_hip.angle(self.hip_up)
            time.sleep(0.2)
            
            # Move left legs backward (counter-clockwise rotation)
            self.fl_knee.angle(self.knee_back)
            self.bl_knee.angle(self.knee_back)
            time.sleep(0.2)
            
            # Put left legs down
            self.fl_hip.angle(self.hip_down)
            self.bl_hip.angle(self.hip_down)
            time.sleep(0.2)
            
            # Return to neutral
            self.stand_neutral()
            time.sleep(0.1)
            
    def autonomous_walk(self):
        """Autonomous walking with obstacle avoidance"""
        print("Starting autonomous mode...")
        print("Press Ctrl+C to stop")
        
        try:
            while True:
                # Check for obstacles
                if self.check_obstacle():
                    print("Obstacle ahead! Turning right...")
                    self.turn_right(2)  # Turn 2 steps to the right
                    time.sleep(0.5)
                else:
                    # No obstacle, walk forward
                    self.walk_forward(1)
                    
        except KeyboardInterrupt:
            print("\nStopping robot...")
            self.stand_neutral()


# ========== Main Program ==========
def main():
    """Main program execution"""
    print("="*50)
    print("Quadruped Spider Robot - Starting...")
    print("="*50)
    
    # Create robot instance
    robot = QuadrupedRobot()
    time.sleep(2)  # Wait for initialization
    
    # Start autonomous walking mode
    robot.autonomous_walk()
    
    # Alternative: Manual control examples (comment out autonomous_walk() above)
    # robot.walk_forward(3)      # Walk 3 steps forward
    # robot.turn_right(2)        # Turn right
    # robot.turn_left(2)         # Turn left
    # robot.stand_neutral()      # Return to neutral position


# Run the main program
if __name__ == "__main__":
    main()