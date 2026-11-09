function [tx, count, heading, roll, pitch, valid, calibration, errorCode] = bno055_controller(rx, rxLength, rxStatus)
% BNO055 UART transaction engine. Call exactly once every 0.001 seconds.
% Nonblocking; all requests wait for a complete response or a bounded timeout.
% Register map/protocol: Bosch BST-BNO055-DS000-18, sections 4.3 and 4.7.
% Stages: ID, CONFIG, page 0, normal power, degrees, NDOF, Euler, calibration.
% Error codes: Bosch EE status, 250=timeout, 251=unexpected reply,
% 252=wrong chip ID, 253=bad payload length, 254=UART receive error.
%#codegen
persistent s
if isempty(s)
    s = struct('delay',1000,'stage',1,'waiting',false,'timeout',0, ...
        'parse',0,'index',0,'need',0,'payload',zeros(1,6,'uint8'), ...
        'attempts',0,'h',0,'r',0,'p',0,'cal',uint8(0),'err',uint8(0), ...
        'age',1000000,'configured',false);
end
tx = zeros(1,5,'uint8');
count = uint8(0);
s.age = min(s.age + 1, 1000000);

if s.waiting && rxLength == 1
    if rxStatus ~= 0
        s = failed(s, uint8(254));
    else
        value = uint8(rx(1));
        if s.parse == 0
            if value == uint8(187) % BB: read response
                s.parse = 1;
            elseif value == uint8(238) % EE: acknowledgement/error
                s.parse = 3;
            end
        elseif s.parse == 1
            expected = 1;
            if s.stage == 7
                expected = 6;
            end
            if (s.stage == 1 || s.stage == 7 || s.stage == 8) && double(value) == expected
                s.need = expected;
                s.index = 0;
                s.parse = 2;
            else
                s = failed(s, uint8(253));
            end
        elseif s.parse == 2
            s.index = s.index + 1;
            s.payload(s.index) = value;
            if s.index == s.need
                if s.stage == 1
                    if s.payload(1) == uint8(160)
                        s = succeeded(s, 2, 2);
                    else
                        s = failed(s, uint8(252));
                    end
                elseif s.stage == 7
                    s.h = signedAngle(s.payload(1), s.payload(2));
                    s.r = signedAngle(s.payload(3), s.payload(4));
                    s.p = signedAngle(s.payload(5), s.payload(6));
                    s.age = 0;
                    s.err = uint8(0);
                    s = succeeded(s, 8, 2);
                elseif s.stage == 8
                    s.cal = s.payload(1);
                    s = succeeded(s, 7, 40);
                end
            end
        else % EE status byte
            if value == uint8(1) && s.stage >= 2 && s.stage <= 6
                previous = s.stage;
                s = succeeded(s, previous + 1, 2);
                if previous == 2
                    s.delay = 30; % mode -> CONFIG: allow >19 ms
                elseif previous == 4
                    s.delay = 10;
                elseif previous == 6
                    s.configured = true;
                    s.delay = 500; % fusion startup, >7 ms mode-switch delay
                end
            elseif value == uint8(1)
                s = failed(s, uint8(251));
            else
                s = failed(s, value);
            end
        end
    end
end

if s.waiting
    s.timeout = s.timeout - 1;
    if s.timeout <= 0
        s = failed(s, uint8(250));
    end
elseif s.delay > 0
    s.delay = s.delay - 1;
else
    if s.stage == 1
        tx(1:4) = uint8([170 1 0 1]);
        count = uint8(4);
    elseif s.stage == 2
        tx = uint8([170 0 61 1 0]); % CONFIG mode
        count = uint8(5);
    elseif s.stage == 3
        tx = uint8([170 0 7 1 0]); % register page 0
        count = uint8(5);
    elseif s.stage == 4
        tx = uint8([170 0 62 1 0]); % normal power
        count = uint8(5);
    elseif s.stage == 5
        tx = uint8([170 0 59 1 0]); % UNIT_SEL: Euler degrees
        count = uint8(5);
    elseif s.stage == 6
        tx = uint8([170 0 61 1 12]); % NDOF fusion
        count = uint8(5);
    elseif s.stage == 7
        tx(1:4) = uint8([170 1 26 6]); % heading, roll, pitch
        count = uint8(4);
    else
        tx(1:4) = uint8([170 1 53 1]); % CALIB_STAT
        count = uint8(4);
    end
    s.waiting = true;
    s.timeout = 150;
    s.parse = 0;
end

heading = s.h;
roll = s.r;
pitch = s.p;
valid = s.configured && s.age < 300;
calibration = s.cal;
errorCode = s.err;
end

function s = succeeded(s, nextStage, delay)
s.waiting = false;
s.timeout = 0;
s.parse = 0;
s.index = 0;
s.attempts = 0;
s.stage = nextStage;
s.delay = delay;
end

function s = failed(s, errorCode)
s.err = errorCode;
s.waiting = false;
s.parse = 0;
s.index = 0;
s.attempts = s.attempts + 1;
s.delay = 20;
if s.attempts >= 3
    s.stage = 1;
    s.delay = 1000;
    s.attempts = 0;
    s.configured = false;
    s.age = 1000000;
    s.cal = uint8(0);
end
end

function value = signedAngle(lo, hi)
value = double(lo) + 256 * double(hi);
if value >= 32768
    value = value - 65536;
end
value = value / 16;
end
