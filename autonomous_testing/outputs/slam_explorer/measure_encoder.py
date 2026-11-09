#!/usr/bin/env python3
"""Powered straight encoder-scale test; stops before asking for measured travel."""
import argparse
import csv
import math
import time
from pathlib import Path
from robot_explorer.vehicle_control import Control
from robot_explorer.core import telemetry,wrap_delta


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-drive-test',action='store_true',help='Explicitly run both motors at negative 50 percent for 3 s')
    parser.add_argument('--port',default='/dev/ttyACM0')
    parser.add_argument('--seconds',type=float,default=3.)
    args=parser.parse_args()
    if not args.run_drive_test:parser.error('Powered test requires --run-drive-test; stop all serial owners first')
    if not .5<=args.seconds<=5:parser.error('Duration must be .5..5 seconds')
    import serial
    c=Control();c.settings.update(motor1=-1,motor2=-1)
    pending=bytearray();next_tx=0;last={};previous=None;delta=[0,0];samples=[];record=False
    started=time.monotonic()
    with serial.Serial(args.port,115200,timeout=.02,write_timeout=.1,exclusive=True) as port:
        port.reset_input_buffer()
        def pump():
            nonlocal pending,next_tx,last,previous
            now=time.monotonic()
            event=c.check_link(now)
            if event:raise RuntimeError(event)
            if now>=next_tx:
                packet=c.next_frame()
                if port.write(packet)!=len(packet):raise RuntimeError('Short write')
                next_tx=now+.1
            pending.extend(port.read(min(max(1,port.in_waiting),4096)))
            while b'\n' in pending:
                raw,_,pending=pending.partition(b'\n')
                line=raw.decode('ascii',errors='replace').strip()
                try:f=telemetry(line)
                except (KeyError,ValueError):continue
                c.receive(line,now);last=f
                counts=[int(f['E1']),int(f['E2'])]
                if record and previous is not None:
                    for i in range(2):delta[i]+=wrap_delta(counts[i],previous[i])
                previous=counts
                samples.append([now-started,*counts,f['D1'],f['D2']])
                if int(f['CMDERR']):raise RuntimeError('Board fault '+line)
        def run(seconds,armed=False):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                pump()
                if armed and c.state!='armed':raise RuntimeError('Unexpected disarm')
        try:
            print('Starts in 2 seconds: both -50%, for',args.seconds,'seconds. Ctrl+C stops.')
            run(2)
            c.request_arm(time.monotonic())
            while c.state=='arming':pump()
            if c.state!='armed':raise RuntimeError('Arm failed')
            record=True;c.drive(50);run(args.seconds,True);c.stop();run(2)
        finally:
            c.stop()
            for _ in range(3):
                try:port.write(c.next_frame());port.flush();time.sleep(.1)
                except Exception:break
    path=Path('encoder_measurement.csv')
    with path.open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['time_s','E1','E2','D1','D2']);w.writerows(samples)
    print('Signed counts including coast:',delta,'saved',path.resolve())
    distance=float(input('Actual measured forward travel from start to full stop (metres): '))
    if not math.isfinite(distance) or not .10<=distance<=10:raise ValueError('Need actual .10..10 m travel')
    for i,d in enumerate(delta,1):
        if abs(d)>10:print('Encoder',i,'counts_per_meter =',abs(d)/distance,'encoder_sign =',1 if d>0 else -1)
    print('Measure steady speed and stopping distance separately; total travel includes coasting.')


if __name__=='__main__':main()
