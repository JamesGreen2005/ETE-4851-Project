function test_pi_motor
addpath(fileparts(mfilename('fullpath')));
assert(motor_crc16(uint8('123456789'))==uint16(hex2dec('29B1')));
clear pi_motor_command
[d,en,armed,e]=pi_motor_command(uint8(0),uint32(0),uint16(1));
assert(d==0 && ~en && ~armed && e==0);

% Motion and ARM are rejected until disarmed-zero establishes the session.
[d,en,~,e]=feed(packet(42,0,2,500,250));
assert(d==0 && ~en && e==6);
[~,en,~,~]=feed(packet(42,1,1,0,250)); assert(~en);
feed(packet(42,2,0,0,250));
[d,en,armed,e]=feed(packet(42,3,1,0,250));
assert(d==0 && en && armed && e==0);

% Heartbeats maintain running duty, bounded by 50% and its slew limit.
seq=4; previous=0;
for k=1:15
    [d,en]=feed(packet(42,seq,2,500,250)); seq=seq+1;
    for t=1:82
        [d,en]=pi_motor_command(uint8(0),uint32(0),uint16(1));
        assert(en && d>=previous-1e-9 && d<=50 && d-previous<=0.95000001);
        previous=d;
    end
end
assert(d==50 && en);

% Manual STOP drops enables immediately on the validated frame.
[d,en]=feed(packet(42,seq,0,0,250)); seq=seq+1;
assert(d==0 && ~en);
feed(packet(42,seq,1,0,250)); seq=seq+1;
feed(packet(42,seq,2,500,250)); seq=seq+1;
for t=1:251
    [d,en,~,e]=pi_motor_command(uint8(0),uint32(0),uint16(1));
end
assert(d==0 && ~en && e==2,'Command timeout did not disable outputs.');
[d,en]=feed(packet(42,seq,2,500,250)); seq=seq+1;
assert(d==0 && ~en,'Run command resumed motion after timeout.');
[~,en]=feed(packet(42,seq,1,0,250)); seq=seq+1;
assert(~en,'Arm without fresh disarm was accepted after timeout.');
feed(packet(42,seq,0,0,250)); seq=seq+1;
feed(packet(42,seq,1,0,250)); seq=seq+1;

% CRC, range, replay, and session faults all disable the bridge.
bad=packet(42,seq,2,500,250); bad(13)=uint8(1);
[d,en,~,e]=feed(bad); assert(d==0 && ~en && e==3);
feed(packet(42,seq,0,0,250)); seq=seq+1;
feed(packet(42,seq,1,0,250)); seq=seq+1;
[~,en,~,e]=feed(packet(42,seq,2,501,250));
assert(~en && e==4);
feed(packet(42,seq,0,0,250)); seq=seq+1;
feed(packet(42,seq,1,0,250)); seq=seq+1;
[~,en,~,e]=feed(packet(42,seq-1,2,200,250)); assert(~en && e==5);
feed(packet(42,seq,0,0,250)); seq=seq+1;
feed(packet(42,seq,1,0,250)); seq=seq+1;
[~,en,~,e]=feed(packet(43,1,2,200,250)); assert(~en && e==6);

% New host must start with disarm; embedded magic bytes in session are legal.
newSession=hex2dec('3CC33CC3');
feed(packet(newSession,0,0,0,250));
[~,en]=feed(packet(newSession,1,1,0,250)); assert(en);
for k=2:6
    [~,en]=feed(packet(newSession,k,2,200,250));
    assert(en,'Embedded magic bytes caused false frame detection.');
end
[d,en,~,e]=pi_motor_command(uint8(0),uint32(0),uint16(2));
assert(d==0 && ~en && e==8);

line=pi_motor_text(123.5,-12.5,45.25,uint8(255),true,uint8(0),true,25,uint8(0));
assert(numel(line)==83);
assert(strcmp(char(line),sprintf('H=+123.5 R=-012.5 P=+045.3 CAL=3/3/3/3 VALID=1 ERR=000 ARM=1 DUTY=25.0 CMDERR=000\r\n')));

% Golden packet also checked independently by Python's binascii.crc_hqx.
gold=packet(hex2dec('12345678'),7,2,250,250);
fprintf('GOLDEN_FRAME_HEX: '); fprintf('%02x',gold); fprintf('\n');
fprintf('PASS: boot stop, explicit arm, slew/cap, stop, timeout latching, CRC, range, replay, session, UART fault, embedded magic, telemetry.\n');
clear pi_motor_command
end

function [d,en,armed,e]=feed(frame)
for k=1:numel(frame)
    [d,en,armed,e]=pi_motor_command(frame(k),uint32(1),uint16(0));
end
end

function bytes=packet(session,seq,op,duty,ttl)
bytes=zeros(1,18,'uint8');
bytes(1:4)=uint8([195 60 1 op]);
bytes(5:8)=u32(session); bytes(9:12)=u32(seq);
bytes(13:14)=uint8([mod(duty,256),floor(duty/256)]);
bytes(15:16)=uint8([mod(ttl,256),floor(ttl/256)]);
crc=motor_crc16(bytes(1:16));
bytes(17:18)=uint8([bitand(crc,uint16(255)),bitshift(crc,-8)]);
end

function b=u32(value)
b=zeros(1,4,'uint8');
for k=1:4, b(k)=uint8(mod(floor(double(value)/256^(k-1)),256)); end
end
