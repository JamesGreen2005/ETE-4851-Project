"""Protocol v5 vehicle control. No camera or LiDAR dependencies."""
import binascii
import math
import struct
from .motor_base import Control as BaseControl, MOTOR


def frame(session, sequence, op, duties=(0, 0), pulses=(0, 0)):
    if not 0 < session <= 0xffffffff or not 0 <= sequence <= 0xffffffff or op not in (0, 1, 2):
        raise ValueError('Invalid command envelope')
    if any(not math.isfinite(v) or abs(v) > 100 for v in duties):
        raise ValueError('Drive duty limit is 100%')
    if op < 2 and any(duties):
        raise ValueError('Arm/stop require zero throttle')
    if (op == 0 and any(pulses)) or (op != 0 and any(not 1000 <= p <= 2000 for p in pulses)):
        raise ValueError('Invalid servo pulses')
    b = struct.pack('<2sBBIIhhHHH', b'\xc3\x3c', 5, op, session, sequence,
                    *(round(d*10) for d in duties), *pulses, 250)
    return b + struct.pack('<H', binascii.crc_hqx(b, 0xffff))


class Control(BaseControl):
    def __init__(self, session=None):
        super().__init__(session)
        self.capable = False
        self.firmware_revision = 0
        self.steering = 0
        self.settings = dict(motor1=1, motor2=1, front=1, rear=-1, travel=500,
                             front_center=1500, rear_center=1500)

    def stop(self):
        super().stop()
        self.steering = 0

    def configure(self, data):
        if self.state != 'disarmed' or not self.confirmed_disarmed:
            raise ValueError('Stop and wait for DISARMED before changing directions')
        updated = self.settings.copy()
        for key in ('motor1', 'motor2', 'front', 'rear'):
            v = data.get(key, updated[key])
            if type(v) is not int or v not in (-1, 1):
                raise ValueError('Direction must be +1 or -1')
            updated[key] = v
        v = data.get('travel', updated['travel'])
        if type(v) is not int or v != 500:
            raise ValueError('Steering travel is locked at 500 us')
        updated['travel'] = v
        for key in ('front_center', 'rear_center'):
            v = data.get(key, updated[key])
            if type(v) is not int or v != 1500:
                raise ValueError('Steering straight position is locked at 1500 us')
            updated[key] = v
        self.settings = updated

    def request_arm(self, now):
        if self.firmware_revision < 2:
            raise ValueError('Install the updated Vehicle_Drive.bin with Upload_Vehicle.ps1; FW=2 servo readback is required for this version')
        if not self.capable:
            raise ValueError('Vehicle_Drive firmware with DRIVE=1 required')
        self.steering = 0
        super().request_arm(now)

    def drive(self, value):
        if self.state != 'armed':
            raise ValueError('Arm first')
        if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 100:
            raise ValueError('Throttle must be between -100 and 100')
        # The matching MCU enforces 500 ms at zero output before reversal.
        # Keep the arm session alive so W/S can change direction during a demo.
        self.duty = value * self.settings['motor1']
        self.duty2 = value * self.settings['motor2']

    def steer(self, value):
        if self.state != 'armed':
            raise ValueError('Arm first')
        if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 1:
            raise ValueError('Steering must be -1..1')
        self.steering = value

    def pulses(self):
        return [round(max(1000, min(2000, self.settings[k+'_center'] +
                                   self.steering*self.settings['travel']*self.settings[k])))
                for k in ('front', 'rear')]

    def next_frame(self):
        op = {'disarmed': 0, 'arming': 1, 'armed': 2}[self.state]
        b = frame(self.session, self.sequence, op,
                  (self.duty, self.duty2) if op == 2 else (0, 0),
                  self.pulses() if op else (0, 0))
        self.sequence += 1
        if op == 0: self.stop_sent = True
        if op == 1: self.arm_sent = True
        return b

    def receive(self, line, now):
        if not line.startswith('H='):
            return None
        if MOTOR.search(line):
            fields = dict(part.split('=',1) for part in line.split() if '=' in part)
            self.firmware_revision = int(fields.get('FW','0'))
            self.capable = ' DRIVE=1 ' in line
            if not self.capable:
                self.last_line = line
                self.stop()
                kind = 'servo/motor bench firmware (SERVO=1)' if ' SERVO=1 ' in line else 'motor-test firmware without DRIVE=1'
                raise RuntimeError('Connected board reports ' + kind + '. Run Upload_Vehicle.ps1 on Windows to install the verified Vehicle_Drive image. Received: ' + line)
            if int(MOTOR.search(line).group(4)) and self.state != 'disarmed':
                self.stop()
                return 'Board command fault — stopped; arm again'
        return super().receive(line, now)
