# File: spider_walk.py
# ESP32 + PCA9685 (I2C) + 8 Servos
# Simulated spider walking gait 🕷️

from machine import Pin, I2C
from time import sleep_ms, ticks_ms
from math import sin, pi

# ───── PCA9685 Driver ─────
class PCA9685:
    def __init__(self, i2c, address=0x40):
        self.i2c = i2c
        self.address = address
        self._write(0x00, 0x00)
        self.set_pwm_freq(50)

    def _write(self, reg, val):
        self.i2c.writeto_mem(self.address, reg, bytes([val]))

    def _write_data(self, reg, data):
        self.i2c.writeto_mem(self.address, reg, data)

    def set_pwm_freq(self, freq):
        prescale = int(25000000.0 / (4096 * freq) - 1)
        oldmode = int.from_bytes(self.i2c.readfrom_mem(self.address, 0x00, 1), 'little')
        newmode = (oldmode & 0x7F) | 0x10
        self._write(0x00, newmode)
        self._write(0xFE, prescale)
        self._write(0x00, oldmode)
        sleep_ms(5)
        self._write(0x00, oldmode | 0xA1)

    def set_pwm(self, channel, on, off):
        data = bytes([on & 0xFF, on >> 8, off & 0xFF, off >> 8])
        self._write_data(0x06 + 4 * channel, data)

# ───── Setup I2C & Driver ─────
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
pwm = PCA9685(i2c)

# ───── Helper Functions ─────
SERVOMIN = 150
SERVOMAX = 600

def angle_to_pwm(angle):
    return int(SERVOMIN + (angle / 180) * (SERVOMAX - SERVOMIN))

# ───── Spider Leg Gait Pattern ─────
def servo_patterns(channel, t):
    """Spider gait: alternate leg pairs with phase shift."""
    speed = 500  # smaller → faster gait

    # Each leg pair (coxa + femur)
    if channel == 0:  # FL Coxa
        return 90 + 40 * sin(t / speed)
    elif channel == 1:  # FL Femur
        return 90 + 30 * sin(t / speed + pi/2)

    elif channel == 2:  # FR Coxa
        return 90 + 40 * sin(t / speed + pi)
    elif channel == 3:  # FR Femur
        return 90 + 30 * sin(t / speed + 3*pi/2)

    elif channel == 4:  # BL Coxa
        return 90 + 40 * sin(t / speed + pi)
    elif channel == 5:  # BL Femur
        return 90 + 30 * sin(t / speed + 3*pi/2)

    elif channel == 6:  # BR Coxa
        return 90 + 40 * sin(t / speed)
    elif channel == 7:  # BR Femur
        return 90 + 30 * sin(t / speed + pi/2)

    else:
        return 90

# ───── Main Loop ─────
print("Spider gait test running...")
while True:
    t = ticks_ms()
    for ch in range(8):
        angle = servo_patterns(ch, t)
        pulse = angle_to_pwm(angle)
        pwm.set_pwm(ch, 0, pulse)
    sleep_ms(20)
