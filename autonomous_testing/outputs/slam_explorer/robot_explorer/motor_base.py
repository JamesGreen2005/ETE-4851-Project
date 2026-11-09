#!/usr/bin/env python3
"""Two-motor protocol-v4 Pi controller; independent forward/reverse duty 0..100%.

Requires pyserial. Heartbeats every 100 ms; MCU command TTL is 250 ms.
Nothing arms automatically. Closing this program sends disarm frames; loss
of the program/cable is handled by the MCU timeout. Does not control old
automatic-start firmware or implement encoder speed regulation.
Servo-test extension: op 3 while drive is disarmed; PA0 front / PB3 rear.
Requires SERVO=1 firmware telemetry. Initial pulse limits are 1000..2000 us.
"""
from __future__ import annotations

import argparse
import binascii
import math
import os
import queue
import re
import secrets
import struct
import sys
import threading
import time

PREFIX = b"\xc3\x3c"
FRAME = struct.Struct("<2sBBIIhhH")
MOTOR = re.compile(r" ARM=([01]) D1=([+-]\d{3}\.\d) D2=([+-]\d{3}\.\d) CMDERR=(\d{3})$")


def command_frame(session: int, sequence: int, op: int, duty: float = 0.0, duty2: float = 0.0) -> bytes:
    if not 0 < session <= 0xFFFFFFFF or not 0 <= sequence <= 0xFFFFFFFF:
        raise ValueError("Invalid session or sequence")
    if op not in (0, 1, 2) or any(not math.isfinite(v) or not -100 <= v <= 100 for v in (duty, duty2)):
        raise ValueError("Signed duty must be finite and between -100 and 100 percent")
    if op != 2 and (duty != 0 or duty2 != 0):
        raise ValueError("Arm/disarm frames must request zero duty")
    body = FRAME.pack(PREFIX, 4, op, session, sequence, round(duty * 10), round(duty2 * 10), 250)
    return body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))


def servo_frame(session: int, sequence: int, pulses) -> bytes:
    """Op 3 uses the two int16 fields as pulse microseconds; zero means off."""
    command_frame(session, sequence, 0)  # Validate envelope.
    if len(pulses) != 2 or any(not math.isfinite(p) or p != int(p) or
                              (p != 0 and not 1000 <= p <= 2000) for p in pulses):
        raise ValueError("Servo pulse must be 0 (off) or an integer 1000..2000 us")
    body = FRAME.pack(PREFIX, 4, 3, session, sequence, *map(int, pulses), 250)
    return body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))


class Control:
    """State and handshake logic separated from I/O for deterministic tests."""

    def __init__(self, session: int | None = None):
        self.session = session if session is not None else secrets.randbelow(0xFFFFFFFF) + 1
        self.sequence = 0
        self.state = "disarmed"
        self.duty = 0.0
        self.duty2 = 0.0
        self.directions = [1, 1]
        self.last_telemetry: float | None = None
        self.last_line = "No telemetry received yet"
        self.arm_started = 0.0
        self.arm_sent = False
        self.stop_sent = False
        self.confirmed_disarmed = False
        self.servo_capable = False
        self.servo_pulses = [0, 0]

    def stop(self):
        self.servo_pulses = [0, 0]
        self.state = "disarmed"
        self.duty = 0.0
        self.duty2 = 0.0
        self.arm_sent = False
        self.stop_sent = False
        self.confirmed_disarmed = False

    def request_arm(self, now: float):
        if any(self.servo_pulses):
            raise ValueError("Use stop before arming drive motors")
        if self.state != "disarmed":
            raise ValueError("Already armed or waiting for arm acknowledgement")
        if not self.confirmed_disarmed or self.last_telemetry is None or now - self.last_telemetry > 0.5:
            raise ValueError("Wait for fresh DISARMED telemetry before arming")
        self.state = "arming"
        self.duty = 0.0
        self.duty2 = 0.0
        self.arm_started = now
        self.arm_sent = False

    def set_speed(self, motor: int, duty: float):
        if self.state != "armed":
            raise ValueError("Type arm and wait for ARMED before setting speed")
        if not math.isfinite(duty) or not 0 <= duty <= 100:
            raise ValueError("Use a duty between 0 and 100 percent")
        if motor not in (1, 2):
            raise ValueError("Motor must be 1 or 2")
        if motor == 1:
            self.duty = duty * self.directions[0]
        else:
            self.duty2 = duty * self.directions[1]

    def set_direction(self, direction: str, motor: int | None = None) -> bool:
        if direction not in ("fwd", "rev") or motor not in (None, 1, 2):
            raise ValueError("Use fwd [1|2] or rev [1|2]; omit motor for both")
        selected = (0, 1) if motor is None else (motor-1,)
        sign = 1 if direction == "fwd" else -1
        changed = any(self.directions[i] != sign for i in selected)
        if changed:
            self.stop()  # Both channels disarm; never resume motion automatically.
            for i in selected:
                self.directions[i] = sign
        return changed

    def set_servo(self, selection: str, pulse: int, now: float):
        if self.state != "disarmed" or not self.confirmed_disarmed:
            raise ValueError("Use stop and wait for DISARMED before testing servos")
        if not self.servo_capable or self.last_telemetry is None or now - self.last_telemetry > 0.5:
            raise ValueError("Fresh servo-capable firmware telemetry required; upload the updated model first")
        if selection not in ("front", "rear", "both"):
            raise ValueError("Use servo front|rear|both center|off|1000..2000")
        candidate = self.servo_pulses.copy()
        for i in ((0, 1) if selection == "both" else (0,) if selection == "front" else (1,)):
            candidate[i] = pulse
        servo_frame(self.session, self.sequence, candidate)
        self.servo_pulses = candidate

    def receive(self, line: str, now: float) -> str | None:
        match = MOTOR.search(line)
        if match is None:
            return None
        self.last_line = line
        self.last_telemetry = now
        self.servo_capable = " SERVO=1 " in line
        board_armed = match.group(1) == "1"
        if any(self.servo_pulses) and (board_armed or not self.servo_capable or
                                     int(match.group(4)) != 0):
            self.stop()
            return "Servo test stopped: board fault, reset, or unexpected motor arm state"
        if self.state == "disarmed" and self.stop_sent and not board_armed:
            if not self.confirmed_disarmed:
                self.confirmed_disarmed = True
                return "DISARMED — ready for an explicit arm command"
        elif self.state == "arming" and self.arm_sent and board_armed:
            self.state = "armed"
            return "ARMED at zero duty — use speed 1 50 or speed 2 50 (maximum 100)"
        elif self.state == "armed" and not board_armed:
            self.stop()
            return "Board disarmed/reset/timed out. Stopped; type arm again when ready."
        return None

    def check_link(self, now: float) -> str | None:
        if (self.state != "disarmed" or any(self.servo_pulses)) and (
            self.last_telemetry is None or now - self.last_telemetry > 0.5
        ):
            self.stop()
            return "Telemetry lost — sending STOP. Manual arm is required to resume."
        if self.state == "arming" and now - self.arm_started > 1.0:
            self.stop()
            return "Arm was not acknowledged — stopped."
        return None

    def next_frame(self) -> bytes:
        if self.sequence > 0xFFFFFFFF:
            raise RuntimeError("Sequence exhausted; stop and restart the controller")
        op = {"disarmed": 0, "arming": 1, "armed": 2}[self.state]
        if any(self.servo_pulses):
            if self.state != "disarmed":
                raise RuntimeError("Servo tests require drive motors disarmed")
            frame = servo_frame(self.session, self.sequence, self.servo_pulses)
        else:
            frame = command_frame(self.session, self.sequence, op, self.duty if op == 2 else 0, self.duty2 if op == 2 else 0)
        self.sequence += 1
        if op == 0:
            self.stop_sent = True
        if op == 1:
            self.arm_sent = True
        return frame


def input_worker(commands):
    while True:
        try:
            command = input("motor> ")
        except EOFError:
            commands.put("quit")
            return
        commands.put(command.strip())
        if command.strip().lower() in ("quit", "exit"):
            return


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    args = parser.parse_args(argv)
    try:
        import serial
    except ImportError:
        print("Install pyserial: sudo apt install python3-serial", file=sys.stderr)
        return 1

    control = Control()
    commands = queue.Queue()
    show_imu = False
    buffer = bytearray()
    ser = None
    try:
        options = {"exclusive": True} if os.name == "posix" else {}
        ser = serial.Serial(args.port, 115200, timeout=0.02, write_timeout=0.1, **options)
        ser.reset_input_buffer()
        print("Dual_Motor_Pi_IMU v4: starts DISARMED, both directions FWD; duty 0–100%.")
        print("Commands: fwd [1|2] | rev [1|2] | arm | speed 1 50 | speed 2 50 | stop | status | imu on/off | quit")
        print("Close miniterm first. Keep a motor-power disconnect within reach.")
        print("Servo test: servo front|rear|both center|off|1000..2000 (pulse us). Motors must stay DISARMED.")
        threading.Thread(target=input_worker, args=(commands,), daemon=True).start()
        next_send = 0.0
        next_print = 0.0
        while True:
            now = time.monotonic()
            while not commands.empty():
                text = commands.get_nowait().lower()
                parts = text.split()
                if not parts:
                    continue
                try:
                    if len(parts) == 3 and parts[0] == "servo":
                        pulse = {"center": 1500, "off": 0}.get(parts[2])
                        if pulse is None:
                            pulse = int(parts[2])
                        control.set_servo(parts[1], pulse, now)
                        next_send = 0
                        print(f"Servo pulses: front={control.servo_pulses[0]} us, rear={control.servo_pulses[1]} us (0=off). Drive motors disabled.")
                    elif parts == ["arm"]:
                        control.request_arm(now)
                        next_send = 0
                        print("Requesting arm at zero duty...")
                    elif parts[0] in ("fwd", "rev") and len(parts) in (1, 2):
                        changed = control.set_direction(parts[0], int(parts[1]) if len(parts) == 2 else None)
                        next_send = 0
                        labels = ["FWD" if d > 0 else "REV" for d in control.directions]
                        print(f"Directions: motor 1 {labels[0]}, motor 2 {labels[1]}")
                        if changed:
                            print("STOP requested for both motors. Wait for DISARMED and physical stop, then arm and set speed again.")
                    elif len(parts) == 3 and parts[0] == "speed":
                        control.set_speed(int(parts[1]), float(parts[2]))
                        next_send = 0
                        print(f"Requested motor 1={control.duty:g}%, motor 2={control.duty2:g}% PWM (not measured RPM)")
                    elif parts == ["stop"]:
                        control.stop()
                        next_send = 0
                        print("STOP requested — drive disabled, servo signals off")
                    elif parts in (["quit"], ["exit"]):
                        return 0
                    elif parts == ["status"]:
                        print(control.last_line)
                        print(f"Servo firmware available: {control.servo_capable}; pulses front/rear: {control.servo_pulses} us (0=off)")
                        print("Selected directions: " + ", ".join(f"motor {i+1} {'FWD' if d>0 else 'REV'}" for i,d in enumerate(control.directions)))
                    elif parts in (["imu", "on"], ["imu", "off"]):
                        show_imu = parts[1] == "on"
                    else:
                        print("Use: servo front|rear|both center|off|1000..2000, fwd [1|2], rev [1|2], arm, speed MOTOR 0..100, stop, status, imu on/off, quit")
                except ValueError as error:
                    print(error)

            event = control.check_link(now)
            if event:
                print(event)
                next_send = 0
            if now >= next_send:
                frame = control.next_frame()
                if ser.write(frame) != len(frame):
                    raise OSError("Incomplete command write")
                next_send = now + 0.1

            buffer.extend(ser.read(min(max(1, ser.in_waiting), 1024)))
            if len(buffer) > 4096:
                buffer.clear()
            while b"\n" in buffer:
                raw, _, remainder = buffer.partition(b"\n")
                buffer = bytearray(remainder)
                line = raw.decode("ascii", errors="replace").strip()
                event = control.receive(line, time.monotonic())
                if event:
                    print(event)
                    if control.state == "disarmed":
                        next_send = 0
                if line.startswith("H=") and MOTOR.search(line) is None:
                    raise RuntimeError("Incompatible firmware detected. Switch motor power OFF and upload the 100%-capable Dual_Motor_Pi_IMU v4 model first.")
                if show_imu and MOTOR.search(line) and now >= next_print:
                    print(line)
                    next_print = now + 1.0
    except KeyboardInterrupt:
        print("\nStopping...")
        return 0
    except (OSError, RuntimeError, serial.SerialException) as error:
        print(f"Connection stopped: {error}. Use the motor-power disconnect if the motor is running.", file=sys.stderr)
        return 1
    finally:
        if ser is not None:
            control.stop()
            for _ in range(3):
                try:
                    ser.write(control.next_frame())
                    ser.flush()
                    time.sleep(0.02)
                except (OSError, serial.SerialException):
                    break
            ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
