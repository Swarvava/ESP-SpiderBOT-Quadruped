# ============================================================
# main.py — Quadruped Spider Robot (ESP32 + MicroPython)
# ============================================================
# Features:
# - 8 servo motors (2 DOF per leg)
# - Ultrasonic obstacle detection
# - Bluetooth mobile control (UART or BLE)
# - Static stability via CoM-support polygon check
# - Walking gait based on research paper’s pose pattern
# ------------------------------------------------------------
# Author: [Your Name]
# Hardware: ESP32, 8× SG90/MG90S servos, 1× HC-SR04
# ============================================================

from machine import Pin, PWM, UART
import ubluetooth
import time, math

# ============================================================
# --------------------- CONFIGURATION -------------------------
# ============================================================

PIN_MAP = {
    'legs': [
        (23, 22),  # Leg 0 - Front Left
        (21, 19),  # Leg 1 - Front Right
        (18, 17),  # Leg 2 - Rear Left
        (16, 15),  # Leg 3 - Rear Right
    ],
    'ultrig': 13,       # Ultrasonic Trigger (GPIO13)
    'ultraecho': 12,    # Ultrasonic Echo (GPIO12)
    'bt_tx': 4,         # Bluetooth TX (GPIO4)
    'bt_rx': 5,         # Bluetooth RX (GPIO5)
    'bt_baud': 9600
}

# Link geometry (meters)
LINK_L1 = 0.15  # Hip → Knee
LINK_L2 = 0.15  # Knee → Foot

# Link center of mass (approx halfway)
LC1 = LINK_L1 / 2
LC2 = LINK_L2 / 2

# Mass estimates (kg)
M_LINK1 = 0.05
M_LINK2 = 0.05
M_BODY  = 0.8

TOTAL_MASS = M_BODY + 4 * (M_LINK1 + M_LINK2)

# Servo setup
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_FREQ   = 50
SERVO_ANGLE_MIN = 0
SERVO_ANGLE_MAX = 180

# Default neutral joint angles
NEUTRAL_HIP  = 90
NEUTRAL_KNEE = 120

# Gait parameters
SAFE_DISTANCE_CM     = 20.0
STEP_DELAY           = 0.12
LIFT_DELTA           = 45
HIP_DELTA            = 25
MAX_ALLOWED_TORQUE_NM = 0.8

# ============================================================
# ------------------- HELPER FUNCTIONS ------------------------
# ============================================================

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def point_in_polygon(x, y, poly):
    """Ray-casting test to check if (x, y) is inside polygon."""
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        intersect = ((yi > y) != (yj > y)) and \
                    (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
        if intersect:
            inside = not inside
        j = i
    return inside


# ============================================================
# ------------------- BLUETOOTH CLASS -------------------------
# ============================================================

class ESP32_Bluetooth:
    """BLE Serial bridge for mobile communication."""

    def __init__(self, name="ESP32_Quadruped"):
        self.bt = ubluetooth.BLE()
        self.bt.active(True)
        self.name = name
        self.bt.irq(self.bt_irq)
        self.register()
        self.advertiser()
        print("Bluetooth initialized as", name)

    def bt_irq(self, event, data):
        if event == 1:
            print("Device connected")
        elif event == 2:
            print("Device disconnected")
            self.advertiser()
        elif event == 3:
            buffer = self.bt.gatts_read(self.rx)
            command = buffer.decode('utf-8').strip()
            print("Received command:", command)
            # TODO: integrate motion logic here if using BLE directly

    def register(self):
        UART_UUID = ubluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
        UART_TX = (ubluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"), ubluetooth.FLAG_NOTIFY,)
        UART_RX = (ubluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"), ubluetooth.FLAG_WRITE,)
        UART_SERVICE = (UART_UUID, (UART_TX, UART_RX))
        services = (UART_SERVICE,)
        ((self.tx, self.rx,),) = self.bt.gatts_register_services(services)

    def advertiser(self):
        name = bytes(self.name, 'utf-8')
        adv_data = bytearray('\x02\x01\x06' + chr(len(name)+1) + '\x09' + self.name, 'utf-8')
        self.bt.gap_advertise(100, adv_data)


# ============================================================
# ------------------- SERVO CLASS -----------------------------
# ============================================================

class Servo:
    """PWM servo driver with angle control."""

    def __init__(self, gpio, freq=SERVO_FREQ, min_us=SERVO_MIN_US, max_us=SERVO_MAX_US):
        self.pin = PWM(Pin(gpio), freq=freq)
        self.freq = freq
        self.min_us = min_us
        self.max_us = max_us
        self.period_us = 1_000_000 // freq

    def angle_to_duty(self, angle):
        angle = clamp(int(angle), SERVO_ANGLE_MIN, SERVO_ANGLE_MAX)
        us = self.min_us + (self.max_us - self.min_us) * (angle / 180.0)
        duty_ratio = us / self.period_us
        try:
            return int(duty_ratio * 65535)
        except:
            return int(duty_ratio * 1023)

    def set_angle(self, angle):
        duty = self.angle_to_duty(angle)
        try:
            self.pin.duty_u16(duty)
        except:
            self.pin.duty(duty)

    def deinit(self):
        try:
            self.pin.deinit()
        except:
            pass


# ============================================================
# ------------------- ULTRASONIC CLASS ------------------------
# ============================================================

class Ultrasonic:
    """HC-SR04 Ultrasonic distance sensor."""

    def __init__(self, trig_pin, echo_pin):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.trig.value(0)
        time.sleep_ms(50)

    def distance_cm(self):
        self.trig.value(0)
        time.sleep_us(2)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)

        try:
            pulse = machine.time_pulse_us(self.echo, 1, 30000)
        except Exception:
            return 999.0

        return pulse / 58.0 if pulse > 0 else 999.0


# ============================================================
# ------------------- KINEMATICS ------------------------------
# ============================================================

def fk_leg(theta1, theta2, L1=LINK_L1, L2=LINK_L2):
    """Forward kinematics: return (x, y) foot position."""
    x = L1 * math.cos(math.radians(theta1)) + L2 * math.cos(math.radians(theta1 + theta2))
    y = L1 * math.sin(math.radians(theta1)) + L2 * math.sin(math.radians(theta1 + theta2))
    return x, y


def ik_leg(x, y, L1=LINK_L1, L2=LINK_L2):
    """Inverse kinematics: given (x, y), return (theta1, theta2)."""
    r2 = x*x + y*y
    r = math.sqrt(r2)
    cos_t2 = (r2 - L1*L1 - L2*L2) / (2 * L1 * L2)
    cos_t2 = clamp(cos_t2, -1.0, 1.0)
    theta2 = math.degrees(math.acos(cos_t2))
    k1 = L1 + L2 * math.cos(math.radians(theta2))
    k2 = L2 * math.sin(math.radians(theta2))
    theta1 = math.degrees(math.atan2(y, x) - math.atan2(k2, k1))
    return theta1, theta2


def gravity_torques(theta1_deg, theta2_deg, m1=M_LINK1, m2=M_LINK2, lc1=LC1, lc2=LC2):
    """Estimate joint torques due to gravity."""
    g = 9.81
    t1 = math.radians(theta1_deg)
    t12 = math.radians(theta1_deg + theta2_deg)
    tau2 = m2 * lc2 * g * math.cos(t12)
    tau1 = (m1 * lc1 * g * math.cos(t1)) + (m2 * g * (LINK_L1 * math.cos(t1) + lc2 * math.cos(t12)))
    return tau1, tau2


# ============================================================
# ------------------- QUADRUPED CLASS -------------------------
# ============================================================

class Quadruped:
    """Main quadruped control class."""

    def __init__(self, pin_map):
        # Servo setup
        self.legs = []
        for hip_pin, knee_pin in pin_map['legs']:
            hip = Servo(hip_pin)
            knee = Servo(knee_pin)
            self.legs.append({'hip': hip, 'knee': knee, 'theta1': NEUTRAL_HIP, 'theta2': NEUTRAL_KNEE})

        # Sensors
        self.ultra = Ultrasonic(pin_map['ultrig'], pin_map['ultraecho'])
        self.bt = UART(2, baudrate=pin_map['bt_baud'],
                       tx=Pin(pin_map['bt_tx']), rx=Pin(pin_map['bt_rx']))

        # Hip anchor positions
        half_x, half_y = 0.09, 0.06
        self.hip_positions = [
            (+half_x, -half_y),
            (+half_x, +half_y),
            (-half_x, -half_y),
            (-half_x, +half_y),
        ]

        self.set_all_neutral()

    # ------------------- Core Functions -------------------

    def set_all_neutral(self):
        """Move all servos to neutral (standing) position."""
        for leg in self.legs:
            leg['theta1'], leg['theta2'] = NEUTRAL_HIP, NEUTRAL_KNEE
            leg['hip'].set_angle(NEUTRAL_HIP)
            leg['knee'].set_angle(NEUTRAL_KNEE)
        time.sleep(0.5)

    def read_bt(self):
        """Read single command from Bluetooth (UART)."""
        if self.bt.any():
            cmd = self.bt.read(1)
            try:
                return cmd.decode().upper()
            except:
                return None
        return None

    def ultrasonic_obstacle(self):
        """Check for obstacle using ultrasonic sensor."""
        d = self.ultra.distance_cm()
        return d < SAFE_DISTANCE_CM if d != 999.0 else False

    # ------------------- Center of Mass -------------------

    def compute_com(self):
        """Compute 2D projection of the robot's center of mass."""
        sum_mx, sum_my, sum_m = M_BODY * 0, M_BODY * 0, M_BODY
        for i, leg in enumerate(self.legs):
            hip_x, hip_y = self.hip_positions[i]
            t1, t12 = math.radians(leg['theta1']), math.radians(leg['theta1'] + leg['theta2'])
            l1cx = hip_x + LC1 * math.cos(t1)
            l1cy = hip_y + LC1 * math.sin(t1)
            l2cx = hip_x + LINK_L1 * math.cos(t1) + LC2 * math.cos(t12)
            l2cy = hip_y + LINK_L1 * math.sin(t1) + LC2 * math.sin(t12)
            sum_mx += M_LINK1 * l1cx + M_LINK2 * l2cx
            sum_my += M_LINK1 * l1cy + M_LINK2 * l2cy
            sum_m += M_LINK1 + M_LINK2
        return sum_mx / sum_m, sum_my / sum_m

    def support_polygon(self, grounded):
        """Compute convex hull polygon from grounded feet."""
        pts = [self.foot_pos_body(i) for i in grounded]
        pts_unique = list(set(pts))
        if len(pts_unique) <= 2:
            return pts_unique

        pts_sorted = sorted(pts_unique)
        def cross(o, a, b):
            return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

        lower, upper = [], []
        for p in pts_sorted:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)
        for p in reversed(pts_sorted):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)
        return lower[:-1] + upper[:-1]

    def com_inside_support(self, grounded):
        """Check if CoM projection is inside the support polygon."""
        hull = self.support_polygon(grounded)
        comx, comy = self.compute_com()
        return point_in_polygon(comx, comy, hull)

    # ------------------- Gait Functions -------------------

    def foot_pos_body(self, leg_idx):
        """Compute each foot’s (x, y) position in body frame."""
        hip_x, hip_y = self.hip_positions[leg_idx]
        theta1 = self.legs[leg_idx]['theta1']
        theta2 = self.legs[leg_idx]['theta2']
        fx, fy = fk_leg(theta1 - NEUTRAL_HIP, theta2 - (NEUTRAL_KNEE - NEUTRAL_HIP))
        return hip_x + fx, hip_y

    def step_leg_safe(self, leg_idx, hip_delta=HIP_DELTA, lift_delta=LIFT_DELTA):
        """Perform single safe step for one leg."""
        grounded = [i for i in range(4) if i != leg_idx]
        if not self.com_inside_support(grounded):
            print("Unstable to lift leg", leg_idx)
            return False

        old_t1, old_t2 = self.legs[leg_idx]['theta1'], self.legs[leg_idx]['theta2']

        # Lift
        self.legs[leg_idx]['knee'].set_angle(clamp(old_t2 - lift_delta, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX))
        time.sleep(STEP_DELAY)

        # Swing
        self.legs[leg_idx]['hip'].set_angle(clamp(old_t1 + hip_delta, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX))
        time.sleep(STEP_DELAY)

        # Lower
        self.legs[leg_idx]['knee'].set_angle(old_t2)
        time.sleep(STEP_DELAY)

        # Push
        self.legs[leg_idx]['hip'].set_angle(clamp(old_t1 - hip_delta//2, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX))
        time.sleep(STEP_DELAY)

        # Restore neutral
        self.legs[leg_idx]['hip'].set_angle(old_t1)
        time.sleep(STEP_DELAY)
        return True

    # ------------------- Torque Check -------------------

    def check_gravity_safety(self):
        """Ensure no servo exceeds safe gravity torque."""
        for i, leg in enumerate(self.legs):
            tau1, tau2 = gravity_torques(leg['theta1'], leg['theta2'])
            if abs(tau1) > MAX_ALLOWED_TORQUE_NM or abs(tau2) > MAX_ALLOWED_TORQUE_NM:
                print("Torque exceed at leg", i)
                return False
        return True

    # ------------------- Movement Primitives -------------------

    def move_forward(self, steps=1):
        order = [0, 2, 1, 3]
        for s in range(steps):
            for leg in order:
                if self.ultrasonic_obstacle() or not self.check_gravity_safety():
                    print("Obstacle or overload — stopping.")
                    return
                self.step_leg_safe(leg)

    def move_backward(self, steps=1):
        order = [1, 3, 0, 2]
        for s in range(steps):
            for leg in order:
                if self.ultrasonic_obstacle():
                    return
                self.step_leg_safe(leg, hip_delta=-HIP_DELTA)

    def turn_left(self, steps=1):
        for s in range(steps):
            for leg in range(4):
                if self.ultrasonic_obstacle():
                    return
                delta = HIP_DELTA//2 if leg % 2 == 0 else -HIP_DELTA//2
                self.step_leg_safe(leg, hip_delta=delta)

    def turn_right(self, steps=1):
        for s in range(steps):
            for leg in range(4):
                if self.ultrasonic_obstacle():
                    return
                delta = -HIP_DELTA//2 if leg % 2 == 0 else HIP_DELTA//2
                self.step_leg_safe(leg, hip_delta=delta)

    def stop(self):
        """Stop motion and return to neutral."""
        self.set_all_neutral()

    def deinit(self):
        for leg in self.legs:
            leg['hip'].deinit()
            leg['knee'].deinit()


# ============================================================
# ------------------- MAIN LOOP ------------------------------
# ============================================================

def run():
    print("Starting Quadruped Controller (MicroPython)")
    robot = Quadruped(PIN_MAP)
    print("BT Commands: F=Forward, B=Back, L=Left, R=Right, S=Stop, N=Neutral")

    try:
        while True:
            if robot.ultrasonic_obstacle():
                print("Obstacle detected. Holding neutral pose.")
                robot.stop()
               
