function test_motor_imu
% Protocol emulator tests; no hardware access or deployment.
addpath(fileparts(mfilename('fullpath')));
clear bno055_controller
queue = uint8([]);
commands = {};
commandTicks = [];
lastValid = false;
for tick = 1:5000
    [byte,n,status,queue] = nextByte(queue);
    [tx,count,h,r,p,valid,cal,err] = bno055_controller(byte,n,status);
    if count > 0
        assert(isempty(queue),'A second request overlapped its preceding response.');
        frame = tx(1:double(count));
        commands{end+1} = frame; %#ok<AGROW>
        commandTicks(end+1) = tick; %#ok<AGROW>
        queue = reply(frame);
    end
    lastValid = valid;
end
assert(lastValid && cal == 255 && err == 0);
assert(h == 123.5 && r == -12.5 && p == 45.25,'Euler order, sign, or scale is wrong.');
assert(isequal(commands{1},uint8([170 1 0 1])));
expected = {uint8([170 0 61 1 0]),uint8([170 0 7 1 0]), ...
    uint8([170 0 62 1 0]),uint8([170 0 59 1 0]),uint8([170 0 61 1 12])};
for k = 1:5
    assert(isequal(commands{k+1},expected{k}),'Initialization sequence is wrong.');
end
assert(commandTicks(3)-commandTicks(2) >= 30,'CONFIG mode delay too short.');
assert(commandTicks(7)-commandTicks(6) >= 500,'Fusion startup delay too short.');
assert(commandTicks(1) >= 1000,'Power-on wait too short.');

% A disconnected sensor must expire VALID, never freeze it true.
for tick = 1:1500
    [~,~,~,~,~,valid,~,err] = bno055_controller(uint8(0),uint32(0),uint16(1));
end
assert(~valid && err == 250,'Missing replies did not time out and expire freshness.');

% Recover with sensor reconnected, including fragmented/garbage frames.
queue = uint8([]);
injected = false;
for tick = 1:5000
    [byte,n,status,queue] = nextByte(queue);
    [tx,count,h,r,p,valid,cal,err] = bno055_controller(byte,n,status);
    if count > 0
        frame = tx(1:double(count));
        queue = reply(frame);
        if ~injected && frame(2) == 1 && frame(3) == 26
            queue = uint8([187 7 0 0 0]); % deliberately wrong frame length
            injected = true;
        end
    end
end
assert(injected && valid && err == 0 && cal == 255 && h == 123.5 && r == -12.5 && p == 45.25);

% A different chip ID must not initialize sensor fusion.
clear bno055_controller
queue = uint8([]);
for tick = 1:3000
    [byte,n,status,queue] = nextByte(queue);
    [tx,count,~,~,~,valid,~,err] = bno055_controller(byte,n,status);
    if count > 0
        assert(tx(2) == 1 && tx(3) == 0,'Wrong chip ID was accepted.');
        queue = uint8([187 1 42]);
    end
end
assert(~valid && err == 252);

line = bno055_text(123.5,-12.5,45.25,uint8(255),true,uint8(0));
assert(numel(line) == 56 && isa(line,'uint8'));
assert(strcmp(char(line),sprintf('H=+123.5 R=-012.5 P=+045.3 CAL=3/3/3/3 VALID=1 ERR=000\r\n')));
line = bno055_text(0,0,0,uint8(27),false,uint8(250));
assert(strcmp(char(line),sprintf('H=+000.0 R=+000.0 P=+000.0 CAL=0/1/2/3 VALID=0 ERR=250\r\n')));
fprintf('PASS: initialization, mode delays, signed Euler decoding, calibration, timeouts, reconnection, malformed frames, chip ID rejection, ASCII telemetry.\n');
clear bno055_controller
end

function [byte,n,status,q] = nextByte(q)
byte = uint8(0); n = uint32(0); status = uint16(1);
if ~isempty(q)
    byte = q(1); q(1) = []; n = uint32(1); status = uint16(0);
end
end

function response = reply(frame)
if frame(2) == 0
    response = uint8([238 1]);
elseif frame(3) == 0
    response = uint8([187 1 160]);
elseif frame(3) == 26
    response = uint8([187 6 184 7 56 255 212 2]);
elseif frame(3) == 53
    response = uint8([187 1 255]);
else
    error('Unexpected register request.');
end
end
