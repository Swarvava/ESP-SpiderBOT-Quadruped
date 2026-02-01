# main.py
# Quadruped spider (8-servo, 2-DOF/leg) MicroPython controller
# - IK, Jacobian, gravity torque estimate
# - Whole-robot CoM check (support-polygon containment)
# - One-leg-at-a-time creeping gait (statical stability)
# - Ultrasonic obstacle stop
# - Bluetooth (UART) single-letter remote control
#
# Important: Calibrate PIN_MAP, LINK lengths, masses, NEUTRAL_ANGLES before use.
# I used the walking-pattern & CoM ideas from the research paper you uploaded as the stability backbone. :contentReference[oaicite:3]{index=3} :contentReference[oaicite:4]{index=4}
from machine import Pin
import ubluetooth
import time

class ESP32_Bluetooth:
    def __init__(self, name="ESP32_Quadruped"):
        self.bt = ubluetooth.BLE()
        self.bt.active(True)
        self.name = name
        self.bt.irq(self.bt_irq)
        self.register()
        self.advertiser()
        print("Bluetooth initialized as", name)

    def bt_irq(self, event, data):
        if event == 1:  # Central connected
            print("Device connected")
        elif event == 2:  # Central disconnected
            print("Device disconnected")
            self.advertiser()
        elif event == 3:  # Write request
            buffer = self.bt.gatts_read(self.rx)
            command = buffer.decode('utf-8').strip()
            print("Received command:", command)
            # TODO: Add servo motion logic here

    def register(self):
        UART_UUID = ubluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
        UART_TX = (ubluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"), ubluetooth.FLAG_NOTIFY,)
        UART_RX = (ubluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"), ubluetooth.FLAG_WRITE,)
        UART_SERVICE = (UART_UUID, (UART_TX, UART_RX))
        services = (UART_SERVICE,)
        ((self.tx, self.rx,), ) = self.bt.gatts_register_services(services)

    def advertiser(self):
        name = bytes(self.name, 'utf-8')
        adv_data = bytearray('\x02\x01\x06' + chr(len(name)+1) + '\x09' + self.name, 'utf-8')
        self.bt.gap_advertise(100, adv_data)

# Start Bluetooth
bt = ESP32_Bluetooth("SpiderBot")
while True:
    time.sleep(1)

import time
import math
from machine import Pin, PWM, UART

# ---------------------------
# ====== USER CONFIG ========
# ---------------------------
# For clarity I show GPIO and NodeMCU-style D labels in comments.
PIN_MAP = {
    'legs': [
        # (hip_gpio, knee_gpio)  # NodeMCU-style "D" label as comment
        (23, 22),  # leg0 front-left  (GPIO23 D23, GPIO22 D22)
        (21, 19),  # leg1 front-right (GPIO21 D21, GPIO19 D19)
        (18, 17),  # leg2 rear-left   (GPIO18 D18, GPIO17 D17)
        (16, 15),  # leg3 rear-right  (GPIO16 D16, GPIO15 D15)
    ],
    'ultrig': 13,   # GPIO13 (D13)
    'ultraecho': 12,# GPIO12 (D12)
    # Bluetooth UART pins (HC-05). Choose pins not used by servos.
    'bt_tx': 4,     # ESP32 TX -> HC-05 RX (GPIO4 D4)
    'bt_rx': 5,     # ESP32 RX <- HC-05 TX (GPIO5 D5)
    'bt_baud': 9600
}

# Link geometry (meters) - change to match your printed parts
LINK_L1 = 0.15  # femur (hip->knee), e.g. 150 mm
LINK_L2 = 0.15  # tibia (knee->foot)

# Link centers (approx half-length for uniform links)
LC1 = LINK_L1 / 2.0
LC2 = LINK_L2 / 2.0

# Masses (kg) - approximate: link segments + servo mass contribution
M_LINK1 = 0.05
M_LINK2 = 0.05
# Body mass (central chassis) - affects CoM; measure or estimate
M_BODY = 0.8

# Total robot mass (sum of body + 4*(link1+link2))
TOTAL_MASS = M_BODY + 4*(M_LINK1 + M_LINK2)

# Servo / safety parameters
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_FREQ = 50
SERVO_ANGLE_MIN = 0
SERVO_ANGLE_MAX = 180
# Neutral angles (degrees) map to mechanical mounting - tune per build
NEUTRAL_HIP = 90
NEUTRAL_KNEE = 120

# Gait & safety
SAFE_DISTANCE_CM = 20.0   # ultrasonic stop threshold
STEP_DELAY = 0.12         # time between micro-movements
LIFT_DELTA = 45           # knee angle delta (degrees) to lift foot
HIP_DELTA = 25            # hip swing in degrees for step length
MAX_ALLOWED_TORQUE_NM = 0.8  # safety ceiling: do not command pose that requires > this torque per joint

# ---------------------------
# ======= UTILITIES =========
# ---------------------------
def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

# Ray-casting point-in-polygon test
def point_in_polygon(x, y, poly):
    # poly: list of (x,y) vertices, assumed closed or not (works either way)
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

# ---------------------------
# ======= Servo class =======
# ---------------------------
class Servo:
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
        # try duty_u16 if available; machine.PWM on some ports supports duty_u16
        try:
            duty = int(duty_ratio * 65535)
            return clamp(duty, 0, 65535)
        except:
            duty = int(duty_ratio * 1023)
            return clamp(duty, 0, 1023)

    def set_angle(self, angle):
        duty = self.angle_to_duty(angle)
        # choose method
        try:
            self.pin.duty_u16(duty)
        except:
            try:
                self.pin.duty(duty)
            except:
                pass

    def deinit(self):
        try:
            self.pin.deinit()
        except:
            pass

# ---------------------------
# ==== Ultrasonic helper ====
# ---------------------------
class Ultrasonic:
    def __init__(self, trig_pin, echo_pin):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.trig.value(0)
        time.sleep_ms(50)

    def distance_cm(self):
        # send pulse
        self.trig.value(0)
        time.sleep_us(2)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)
        # measure echo: returns microseconds, or -2 on timeout on some builds
        try:
            pulse = machine.time_pulse_us(self.echo, 1, 30000)
        except Exception:
            return 999.0
        # convert: distance cm = pulse_us / 58
        return pulse / 58.0 if pulse > 0 else 999.0

# ---------------------------
# ===== Kinematics / IK =====
# ---------------------------
def fk_leg(theta1, theta2, L1=LINK_L1, L2=LINK_L2):
    # returns foot (x,y) in leg frame
    x = L1*math.cos(math.radians(theta1)) + L2*math.cos(math.radians(theta1 + theta2))
    y = L1*math.sin(math.radians(theta1)) + L2*math.sin(math.radians(theta1 + theta2))
    return x, y

def ik_leg(x, y, L1=LINK_L1, L2=LINK_L2):
    # returns (theta1_deg, theta2_deg) using standard 2-link IK
    r2 = x*x + y*y
    r = math.sqrt(r2)
    # clamp cosine to [-1,1]
    cos_theta2 = (r2 - L1*L1 - L2*L2) / (2 * L1 * L2)
    cos_theta2 = clamp(cos_theta2, -1.0, 1.0)
    theta2 = math.degrees(math.acos(cos_theta2))
    # choose elbow-down or elbow-up depending on sign; pick elbow-down (positive sin)
    k1 = L1 + L2 * math.cos(math.radians(theta2))
    k2 = L2 * math.sin(math.radians(theta2))
    theta1 = math.degrees(math.atan2(y, x) - math.atan2(k2, k1))
    return theta1, theta2

# Jacobian (for reference / future velocity control or torque mapping)
def jacobian(theta1_deg, theta2_deg, L1=LINK_L1, L2=LINK_L2):
    t1 = math.radians(theta1_deg)
    t12 = math.radians(theta1_deg + theta2_deg)
    j11 = -L1*math.sin(t1) - L2*math.sin(t12)
    j12 = -L2*math.sin(t12)
    j21 =  L1*math.cos(t1) + L2*math.cos(t12)
    j22 =  L2*math.cos(t12)
    return [[j11, j12], [j21, j22]]

# ---------------------------
# ===== Dynamics (gravity torque) =====
# ---------------------------
def gravity_torques(theta1_deg, theta2_deg, m1=M_LINK1, m2=M_LINK2, lc1=LC1, lc2=LC2):
    # Returns gravity torque vector [tau1, tau2] in Nm for given joint angles (deg)
    t1 = math.radians(theta1_deg)
    t12 = math.radians(theta1_deg + theta2_deg)
    g = 9.81
    # Using the approximated G formulas from 2-link Lagrangian:
    tau2 = m2 * lc2 * g * math.cos(t12)
    tau1 = (m1 * lc1 * g * math.cos(t1)) + (m2 * g * (LINK_L1 * math.cos(t1) + lc2 * math.cos(t12)))
    return tau1, tau2

# ---------------------------
# ===== Robot class =========
# ---------------------------
class Quadruped:
    def __init__(self, pin_map):
        # create servos per leg
        self.legs = []
        for hip_pin, knee_pin in pin_map['legs']:
            hip = Servo(hip_pin)
            knee = Servo(knee_pin)
            self.legs.append({'hip': hip, 'knee': knee, 'theta1': NEUTRAL_HIP, 'theta2': NEUTRAL_KNEE})

        self.ultra = Ultrasonic(pin_map['ultrig'], pin_map['ultraecho'])
        self.bt = UART(2, baudrate=pin_map['bt_baud'], tx=Pin(pin_map['bt_tx']), rx=Pin(pin_map['bt_rx']))

        # foot positions in body coordinates (x,y) for each leg neutral pose
        # coordinate convention: body origin at center, +x forward, +y to right
        half_x = 0.09  # forward offset from body center to hip along x (m) - tune
        half_y = 0.06  # lateral offset from center to hip (m) - tune
        self.hip_positions = [
            (+half_x, -half_y),  # leg0 front-left
            (+half_x, +half_y),  # leg1 front-right
            (-half_x, -half_y),  # leg2 rear-left
            (-half_x, +half_y),  # leg3 rear-right
        ]
        # initialize neutral pose
        self.set_all_neutral()

    def set_all_neutral(self):
        for leg in self.legs:
            leg['theta1'] = NEUTRAL_HIP
            leg['theta2'] = NEUTRAL_KNEE
            leg['hip'].set_angle(leg['theta1'])
            leg['knee'].set_angle(leg['theta2'])
        time.sleep(0.5)

    def read_bt(self):
        if self.bt.any():
            cmd = self.bt.read(1)
            try:
                return cmd.decode().upper()
            except:
                return None
        return None

    def ultrasonic_obstacle(self):
        d = self.ultra.distance_cm()
        if d == 999.0:
            return False
        return d < SAFE_DISTANCE_CM

    # compute foot Cartesian positions (body frame) from joint angles
    def foot_pos_body(self, leg_idx):
        hip_body_x, hip_body_y = self.hip_positions[leg_idx]
        theta1 = self.legs[leg_idx]['theta1']
        theta2 = self.legs[leg_idx]['theta2']
        # foot in leg frame
        fx, fy = fk_leg(theta1 - NEUTRAL_HIP, theta2 - (NEUTRAL_KNEE - NEUTRAL_HIP))
        # The above uses a small transform: we assume neutral maps to straight down-ish; you should calibrate mapping
        # For simplicity, place foot relative to hip by adding fx forward and lateral remains hip_body_y
        x = hip_body_x + fx
        y = hip_body_y + 0.0  # planar model, y remains hip offset
        return x, y

    # Compute whole robot CoM in body frame (x,y)
    def compute_com(self):
        # approximate: treat body as mass M_BODY at origin (0,0)
        sum_mx = M_BODY * 0.0
        sum_my = M_BODY * 0.0
        sum_m = M_BODY
        # add leg segments center contributions (projected to body frame)
        for i, leg in enumerate(self.legs):
            hip_x, hip_y = self.hip_positions[i]
            # approximate link1 com location in body frame: hip + (L1/2 along forward direction determined by hip angle)
            t1 = math.radians(leg['theta1'])
            t12 = math.radians(leg['theta1'] + leg['theta2'])
            # link1 center position
            l1cx = hip_x + LC1 * math.cos(t1)
            l1cy = hip_y + LC1 * math.sin(t1)
            # link2 center position
            l2cx = hip_x + LINK_L1 * math.cos(t1) + LC2 * math.cos(t12)
            l2cy = hip_y + LINK_L1 * math.sin(t1) + LC2 * math.sin(t12)
            sum_mx += M_LINK1 * l1cx + M_LINK2 * l2cx
            sum_my += M_LINK1 * l1cy + M_LINK2 * l2cy
            sum_m += M_LINK1 + M_LINK2
        com_x = sum_mx / sum_m
        com_y = sum_my / sum_m
        return com_x, com_y

    # Support polygon creation from grounded feet (list of leg idxs). We use convex hull of foot points simple method
    def support_polygon(self, grounded):
        pts = [self.foot_pos_body(i) for i in grounded]
        # compute convex hull (Andrew monotone chain) in 2D
        pts_unique = list(set(pts))
        if len(pts_unique) <= 2:
            return pts_unique
        pts_sorted = sorted(pts_unique)
        def cross(o, a, b):
            return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
        lower = []
        for p in pts_sorted:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)
        upper = []
        for p in reversed(pts_sorted):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)
        hull = lower[:-1] + upper[:-1]
        return hull

    def com_inside_support(self, grounded):
        hull = self.support_polygon(grounded)
        comx, comy = self.compute_com()
        return point_in_polygon(comx, comy, hull)

    # single leg step with CoM safety
    def step_leg_safe(self, leg_idx, hip_delta=HIP_DELTA, lift_delta=LIFT_DELTA):
        # check that swapping this leg into swing keeps COM in support polygon with other grounded legs
        grounded = [i for i in range(4) if i != leg_idx]
        # ensure current CoM in support before starting
        if not self.com_inside_support(grounded + [leg_idx]):
            print("Warning: initial COM is not inside full-support polygon")
        # Before lifting verify that CoM stays inside support polygon while that leg is lifted (so polygon = grounded)
        if not self.com_inside_support(grounded):
            print("Would be unstable to lift leg", leg_idx, "- skipping")
            return False
        # perform step: lift -> swing -> lower -> push -> neutral
        # lift: increase knee angle (smaller knee value = lift depending on mounting); here we treat higher knee angle as folded
        old_t1 = self.legs[leg_idx]['theta1']
        old_t2 = self.legs[leg_idx]['theta2']
        # Lift
        self.legs[leg_idx]['theta2'] = clamp(old_t2 - lift_delta, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX)
        self.legs[leg_idx]['knee'].set_angle(self.legs[leg_idx]['theta2'])
        time.sleep(STEP_DELAY)
        # swing forward (hip increase)
        self.legs[leg_idx]['theta1'] = clamp(old_t1 + hip_delta, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX)
        self.legs[leg_idx]['hip'].set_angle(self.legs[leg_idx]['theta1'])
        time.sleep(STEP_DELAY)
        # lower
        self.legs[leg_idx]['theta2'] = old_t2
        self.legs[leg_idx]['knee'].set_angle(old_t2)
        time.sleep(STEP_DELAY)
        # push: move hip backward a bit (to create forward body shift)
        self.legs[leg_idx]['theta1'] = clamp(old_t1 - hip_delta//2, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX)
        self.legs[leg_idx]['hip'].set_angle(self.legs[leg_idx]['theta1'])
        time.sleep(STEP_DELAY)
        # restore neutral
        self.legs[leg_idx]['theta1'] = old_t1
        self.legs[leg_idx]['hip'].set_angle(old_t1)
        time.sleep(STEP_DELAY)
        return True

    # check gravity torques across joints and abort if any exceed safety limit
    def check_gravity_safety(self):
        for i, leg in enumerate(self.legs):
            t1, t2 = leg['theta1'], leg['theta2']
            tau1, tau2 = gravity_torques(t1, t2)
            if abs(tau1) > MAX_ALLOWED_TORQUE_NM or abs(tau2) > MAX_ALLOWED_TORQUE_NM:
                print("Torque exceed at leg", i, "tau1=%.2fNm tau2=%.2fNm" % (tau1, tau2))
                return False
        return True

    # locomotion primitives
    def move_forward(self, steps=1):
        # stable order: 0,2,1,3 (alternating diagonals) — from typical statically stable sequences
        order = [0, 2, 1, 3]
        for s in range(steps):
            for leg in order:
                if self.ultrasonic_obstacle():
                    print("Obstacle! stop.")
                    return
                if not self.check_gravity_safety():
                    print("High torque - slowing/stopping")
                    return
                ok = self.step_leg_safe(leg)
                if not ok:
                    print("Skipped leg", leg, "due to instability")
                    time.sleep(0.05)

    def move_backward(self, steps=1):
        order = [1, 3, 0, 2]
        for s in range(steps):
            for leg in order:
                if self.ultrasonic_obstacle():
                    print("Obstacle! stop.")
                    return
                if not self.check_gravity_safety():
                    return
                # do the reverse swing
                ok = self.step_leg_safe(leg, hip_delta=-HIP_DELTA, lift_delta=LIFT_DELTA)
                if not ok:
                    time.sleep(0.05)

    def turn_left(self, steps=1):
        for s in range(steps):
            for leg in [0,1,2,3]:
                if self.ultrasonic_obstacle():
                    return
                if leg % 2 == 0:
                    self.step_leg_safe(leg, hip_delta=HIP_DELTA//2)
                else:
                    self.step_leg_safe(leg, hip_delta=-HIP_DELTA//2)

    def turn_right(self, steps=1):
        for s in range(steps):
            for leg in [0,1,2,3]:
                if self.ultrasonic_obstacle():
                    return
                if leg % 2 == 0:
                    self.step_leg_safe(leg, hip_delta=-HIP_DELTA//2)
                else:
                    self.step_leg_safe(leg, hip_delta=HIP_DELTA//2)

    def stop(self):
        self.set_all_neutral()

    def deinit(self):
        for leg in self.legs:
            leg['hip'].deinit()
            leg['knee'].deinit()

# ---------------------------
# ======= Main loop =========
# ---------------------------
def run():
    print("Starting Quadruped controller (MicroPython).")
    robot = Quadruped(PIN_MAP)
    print("Ready. BT commands: F-forward, B-back, L-left, R-right, S-stop, N-neutral")

    try:
        while True:
            # sensor proactive check
            if robot.ultrasonic_obstacle():
                print("Ultrasonic: obstacle. Stopping and holding neutral.")
                robot.stop()
                # flush BT data
                time.sleep(0.5)
                continue

            cmd = robot.read_bt()
            if cmd:
                print("BT:", cmd)
                if cmd == 'F':
                    robot.move_forward(steps=1)
                elif cmd == 'B':
                    robot.move_backward(steps=1)
                elif cmd == 'L':
                    robot.turn_left(steps=1)
                elif cmd == 'R':
                    robot.turn_right(steps=1)
                elif cmd == 'S':
                    robot.stop()
                elif cmd == 'N':
                    robot.set_all_neutral()
                else:
                    print("Unknown command:", cmd)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopping")
    finally:
        robot.deinit()

if __name__ == '__main__':
    run()
