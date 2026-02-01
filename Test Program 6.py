from machine import Pin, I2C
import time
import bluetooth
from micropython import const

# BLE UART Service UUIDs
_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_RX = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")

_FLAG_READ = const(0x0002)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

# PCA9685 Servo Driver
class PCA9685:
    def __init__(self, i2c, addr=0x40):
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(addr, 0x00, b'\x10')  # Sleep
        self.i2c.writeto_mem(addr, 0xFE, b'\x79')  # Set 50Hz
        self.i2c.writeto_mem(addr, 0x00, b'\x20')  # Wake
        
    def set_angle(self, ch, angle):
        pulse = int(150 + (angle * 2.5))  # 150-600 for 0-180°
        self.i2c.writeto_mem(self.addr, 0x06+ch*4, bytes([0, 0, pulse&0xFF, pulse>>8]))

# Simplified BLE UART Service
class BLEUART:
    def __init__(self, name="SpiderBot"):
        self._ble = bluetooth.BLE()
        self._connections = set()
        self._rx_buffer = bytearray()
        
        # Initialize BLE with error handling
        try:
            self._ble.active(False)  # Reset BLE
            time.sleep(0.5)
            self._ble.active(True)   # Activate BLE
            time.sleep(0.5)
            
            # Set device name
            self._ble.config(gap_name=name)
            
            # Register UART service
            ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services((
                (_UART_UUID, (
                    (_UART_TX, _FLAG_READ | _FLAG_NOTIFY,),
                    (_UART_RX, _FLAG_WRITE,),
                )),
            ))
            
            # Set IRQ handler
            self._ble.irq(self._irq)
            
            # Start advertising
            self._advertise(name)
            print(f"BLE initialized as '{name}'")
            
        except Exception as e:
            print(f"BLE init error: {e}")
            # Try simpler initialization
            self._ble.active(True)
            time.sleep(1)
            self._advertise_simple(name)

    def _irq(self, event, data):
        if event == 1:  # Central connected
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("Device connected")
        elif event == 2:  # Central disconnected
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.remove(conn_handle)
            print("Device disconnected")
            self._advertise_simple("SpiderBot")
        elif event == 3:  # Data received
            conn_handle, value_handle = data
            if value_handle == self._rx_handle:
                self._rx_buffer += self._ble.gatts_read(self._rx_handle)

    def _advertise(self, name):
        # Build advertising payload
        payload = bytearray()
        # Flags
        payload += b'\x02\x01\x06'
        # Name
        name_bytes = name.encode()
        payload += bytes([len(name_bytes) + 1, 0x09]) + name_bytes
        # Service UUID
        payload += b'\x11\x07' + bytes(reversed(bytes(_UART_UUID)))
        
        try:
            self._ble.gap_advertise(100000, adv_data=payload[:31])
        except:
            self._advertise_simple(name)
    
    def _advertise_simple(self, name):
        # Simpler advertising for compatibility
        try:
            adv_data = bytearray(b'\x02\x01\x06') + bytearray((len(name) + 1, 0x09)) + name.encode()
            self._ble.gap_advertise(100000, adv_data=adv_data[:31])
        except Exception as e:
            print(f"Advertising error: {e}")

    def read(self):
        if self._rx_buffer:
            data = bytes(self._rx_buffer)
            self._rx_buffer = bytearray()
            try:
                return data.decode('utf-8').strip()
            except:
                return None
        return None

    def write(self, data):
        if self._connections:
            for conn_handle in self._connections:
                try:
                    self._ble.gatts_notify(conn_handle, self._tx_handle, data.encode())
                except:
                    pass

# Quadruped Robot Controller
class SpiderBot:
    def __init__(self):
        print("Initializing SpiderBot...")
        
        # Initialize I2C and servo driver
        try:
            self.i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
            time.sleep(0.1)
            self.driver = PCA9685(self.i2c)
            print("Servo driver initialized")
        except Exception as e:
            print(f"I2C/Servo error: {e}")
        
        # Initialize Bluetooth
        time.sleep(0.5)
        self.bt = BLEUART("SpiderBot")
        
        # Servo mapping [hip, knee] for each leg
        self.legs = {
            'FL': [0, 1],  # Front Left
            'FR': [2, 3],  # Front Right
            'RL': [4, 5],  # Rear Left
            'RR': [6, 7]   # Rear Right
        }
        
        # Neutral positions
        self.neutral = {'hip': 100, 'knee': 90}
        self.step_height = 30
        self.step_length = 25
        
        # Initialize pose
        time.sleep(0.5)
        self.init_pose()
        
    def init_pose(self):
        """Set robot to neutral standing position"""
        try:
            for leg in self.legs.values():
                self.driver.set_angle(leg[0], self.neutral['hip'])
                self.driver.set_angle(leg[1], self.neutral['knee'])
            time.sleep(0.5)
        except:
            pass
        
    def move_leg(self, leg_name, hip_angle, knee_angle, delay=0.01):
        """Move single leg with IK"""
        try:
            leg = self.legs[leg_name]
            self.driver.set_angle(leg[0], hip_angle)
            self.driver.set_angle(leg[1], knee_angle)
            time.sleep(delay)
        except:
            pass
        
    def creep_gait_forward(self):
        """Forward walking using creep gait pattern"""
        sequence = [
            # Phase 1: Move FL forward
            {'FL': (self.neutral['hip']+self.step_length, self.neutral['knee']-self.step_height)},
            {'FL': (self.neutral['hip']+self.step_length, self.neutral['knee'])},
            
            # Phase 2: Move RR forward
            {'RR': (self.neutral['hip']+self.step_length, self.neutral['knee']-self.step_height)},
            {'RR': (self.neutral['hip']+self.step_length, self.neutral['knee'])},
            
            # Phase 3: Move FR forward
            {'FR': (self.neutral['hip']+self.step_length, self.neutral['knee']-self.step_height)},
            {'FR': (self.neutral['hip']+self.step_length, self.neutral['knee'])},
            
            # Phase 4: Move RL forward
            {'RL': (self.neutral['hip']+self.step_length, self.neutral['knee']-self.step_height)},
            {'RL': (self.neutral['hip']+self.step_length, self.neutral['knee'])},
            
            # Phase 5: Body moves forward (all legs push back)
            {'FL': (self.neutral['hip']-self.step_length, self.neutral['knee']),
             'FR': (self.neutral['hip']-self.step_length, self.neutral['knee']),
             'RL': (self.neutral['hip']-self.step_length, self.neutral['knee']),
             'RR': (self.neutral['hip']-self.step_length, self.neutral['knee'])}
        ]
        
        for phase in sequence:
            for leg_name, (hip, knee) in phase.items():
                self.move_leg(leg_name, hip, knee)
            time.sleep(0.05)
            
    def creep_gait_backward(self):
        """Backward walking - reverse of forward"""
        sequence = [
            # Move legs backward one by one
            {'FL': (self.neutral['hip']-self.step_length, self.neutral['knee']-self.step_height)},
            {'FL': (self.neutral['hip']-self.step_length, self.neutral['knee'])},
            
            {'RR': (self.neutral['hip']-self.step_length, self.neutral['knee']-self.step_height)},
            {'RR': (self.neutral['hip']-self.step_length, self.neutral['knee'])},
            
            {'FR': (self.neutral['hip']-self.step_length, self.neutral['knee']-self.step_height)},
            {'FR': (self.neutral['hip']-self.step_length, self.neutral['knee'])},
            
            {'RL': (self.neutral['hip']-self.step_length, self.neutral['knee']-self.step_height)},
            {'RL': (self.neutral['hip']-self.step_length, self.neutral['knee'])},
            
            # Body moves backward
            {'FL': (self.neutral['hip']+self.step_length, self.neutral['knee']),
             'FR': (self.neutral['hip']+self.step_length, self.neutral['knee']),
             'RL': (self.neutral['hip']+self.step_length, self.neutral['knee']),
             'RR': (self.neutral['hip']+self.step_length, self.neutral['knee'])}
        ]
        
        for phase in sequence:
            for leg_name, (hip, knee) in phase.items():
                self.move_leg(leg_name, hip, knee)
            time.sleep(0.05)
    
    def run(self):
        """Main control loop"""
        print("SpiderBot Ready!")
        print("Waiting for Bluetooth connection...")
        print("Connect using 'Serial Bluetooth Terminal' app")
        print("Look for device: SpiderBot")
        
        time.sleep(1)
        self.bt.write("SpiderBot Ready!\n")
        self.bt.write("Commands: F-Forward, B-Backward, S-Stop\n")
        
        moving = False
        direction = None
        
        while True:
            try:
                cmd = self.bt.read()
                
                if cmd:
                    cmd = cmd.upper()
                    if 'F' in cmd:
                        moving = True
                        direction = 'F'
                        self.bt.write("Moving Forward\n")
                        print("Moving Forward")
                    elif 'B' in cmd:
                        moving = True
                        direction = 'B'
                        self.bt.write("Moving Backward\n")
                        print("Moving Backward")
                    elif 'S' in cmd:
                        moving = False
                        self.init_pose()
                        self.bt.write("Stopped\n")
                        print("Stopped")
                
                if moving:
                    if direction == 'F':
                        self.creep_gait_forward()
                    elif direction == 'B':
                        self.creep_gait_backward()
                
                time.sleep(0.01)
                
            except KeyboardInterrupt:
                print("Stopping SpiderBot...")
                break
            except Exception as e:
                print(f"Runtime error: {e}")
                time.sleep(0.1)

# Main execution
if __name__ == "__main__":
    try:
        # Add startup delay
        print("Starting in 3 seconds...")
        time.sleep(3)
        
        robot = SpiderBot()
        robot.run()
        
    except Exception as e:
        print(f"Fatal Error: {e}")
        print("Trying to reset BLE...")
        try:
            ble = bluetooth.BLE()
            ble.active(False)
            time.sleep(1)
        except:
            pass  