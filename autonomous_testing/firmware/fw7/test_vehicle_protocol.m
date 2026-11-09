function test_vehicle_protocol
clear vehicle_command
[d,en,~,e,p]=feed(packet(42,0,0,0,0,0,0));
assert(~en && all(d==0) && all(p==0) && e==0);
[d,en,~,e,p]=feed(packet(42,1,1,0,0,1500,1500));
assert(en && all(d==0) && isequal(p,[1500 1500]) && e==0);
seq=2;
for k=1:12
    [d,en,~,e,p]=feed(packet(42,seq,2,500,-500,1000,2000)); seq=seq+1;
    for j=1:80, [d,en]=vehicle_command(uint8(0),uint32(0),uint16(1)); end
    assert(en && d(1)>=0 && d(2)<=0 && all(abs(d)<=50));
end
assert(isequal(d,[50 -50]) && isequal(p,[1000 2000]));
zeroRun=[0 0]; reversed=[false false];
for k=1:30
    bytes=packet(42,seq,2,-500,500,1600,1400); seq=seq+1;
    for j=1:100
        if j<=24, [d,en]=vehicle_command(bytes(j),uint32(1),uint16(0));
        else, [d,en]=vehicle_command(uint8(0),uint32(0),uint16(1)); end
        assert(en);
        for m=1:2
            if d(m)==0, zeroRun(m)=zeroRun(m)+1; end
            if (m==1 && d(m)<0)||(m==2 && d(m)>0)
                if ~reversed(m), assert(zeroRun(m)>=500); reversed(m)=true; end
            end
        end
    end
end
assert(all(reversed));
for j=1:251, [d,en,~,e,p]=vehicle_command(uint8(0),uint32(0),uint16(1)); end
assert(~en && all(d==0) && all(p==0) && e==2);
[~,en]=feed(packet(42,seq,1,0,0,1500,1500)); assert(~en);
for fault=1:8
    clear vehicle_command
    feed(packet(42,0,0,0,0,0,0));feed(packet(42,1,1,0,0,1500,1500));
    bytes=packet(42,2,2,250,-250,1500,1500); expected=0;
    switch fault
        case 1, bytes=packet(42,2,2,1001,0,1500,1500);expected=4;
        case 2, bytes=packet(42,2,2,0,0,999,1500);expected=4;
        case 3, bytes=packet(42,2,2,0,0,1500,2001);expected=4;
        case 4, bytes=packet(42,1,2,0,0,1500,1500);expected=5;
        case 5, bytes=packet(43,2,2,0,0,1500,1500);expected=6;
        case 6, bytes(23)=bitxor(bytes(23),uint8(1));expected=3;
        case 7, bytes=packet(42,2,0,0,0,0,0);
        case 8
            [d,en,~,e,p]=vehicle_command(uint8(0),uint32(0),uint16(2));
            assert(~en && all(d==0) && all(p==0) && e==8);continue
    end
    [d,en,~,e,p]=feed(bytes);assert(~en && all(d==0) && all(p==0) && e==expected);
end
% Both 100% polarities reachable; stop and bounds remain enforced.
for polarity=[1 -1]
    clear vehicle_command
    feed(packet(42,0,0,0,0,0,0));feed(packet(42,1,1,0,0,1500,1500));
    previous=[0 0]; seq=2;
    for k=1:24
        bytes=packet(42,seq,2,polarity*1000,-polarity*1000,1600,1400);seq=seq+1;
        for j=1:100
            if j<=24,[d,en]=vehicle_command(bytes(j),uint32(1),uint16(0));
            else,[d,en]=vehicle_command(uint8(0),uint32(0),uint16(1));end
            assert(en && all(abs(d)<=100));
            if k==1 && j==24, assert(isequal(d,polarity*[100 -100])); end
            previous=d;
        end
    end
    assert(isequal(d,polarity*[100 -100]));
    [d,en,~,~,p]=feed(packet(42,seq,0,0,0,0,0));assert(~en && all(d==0) && all(p==0));
end
% Explicit throttle isolation: servo targets must remain exact for every tick.
clear vehicle_command
feed(packet(42,0,0,0,0,0,0));feed(packet(42,1,1,0,0,1475,1525));
seq=2;
for throttle=[0 1000 0 -1000 500 -500 0]
    bytes=packet(42,seq,2,throttle,-throttle,1475,1525);seq=seq+1;
    for j=1:100
        if j<=24,[~,~,~,~,p]=vehicle_command(bytes(j),uint32(1),uint16(0));
        else,[~,~,~,~,p]=vehicle_command(uint8(0),uint32(0),uint16(1));end
        assert(isequal(p,[1475 1525]));
    end
end
text=vehicle_text(0,0,0,uint8(0),true,uint8(0),true,[100 -100],uint8(0),uint16(0),uint16(0),uint16(1475),uint16(1525),uint16(20000));
assert(contains(char(text),'DRIVE=1 FW=2 S1=1475 S2=1525 T2=20000'));
assert(contains(char(text),'INA=1 SH1=+00000 BV1=00000 ST1=0 SH2=+00000 BV2=00000 ST2=0'));
assert(numel(text)==209 && numel(text)<256,'Readback telemetry exceeds configured transmit capacity.');
fprintf('PASS: throttle changes leave servo targets unchanged on every MCU tick; servo readback telemetry.\n');
fprintf('PASS: Vehicle v5 simultaneous drive/steering, 100%% both directions with slew, bounds, reversal interval, timeout latch, stop, CRC/session/replay/UART faults.\n');
fprintf('VEHICLE_GOLDEN: %s\n',sprintf('%02x',packet(42,2,2,250,-250,1600,1400)));
clear vehicle_command
end
function [d,en,a,e,p]=feed(b)
for k=1:numel(b), [d,en,a,e,p]=vehicle_command(b(k),uint32(1),uint16(0)); end
end
function b=packet(session,seq,op,d1,d2,f,r)
b=zeros(1,24,'uint8');b(1:4)=uint8([195 60 5 op]);
for k=1:4
 b(4+k)=uint8(mod(floor(double(session)/256^(k-1)),256));
 b(8+k)=uint8(mod(floor(double(seq)/256^(k-1)),256));
end
v=[mod(d1,65536),mod(d2,65536),f,r,250];
for k=1:5,b(11+2*k)=uint8(mod(v(k),256));b(12+2*k)=uint8(floor(v(k)/256));end
crc=vehicle_crc16(b(1:22));b(23:24)=uint8([bitand(crc,uint16(255)),bitshift(crc,-8)]);
end
