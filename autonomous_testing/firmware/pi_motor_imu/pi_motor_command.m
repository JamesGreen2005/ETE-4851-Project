function [duty, enable, armed, commandError] = pi_motor_command(rx, rxLength, rxStatus)
% Call every 1 ms. Boot disabled; CRC-checked, session/sequence protected.
% Frame: C3 3C, version=1, op, session u32 LE, seq u32 LE,
% duty in 0.1% u16 LE (0..500), TTL ms u16 LE (50..250), CRC16 LE.
% CRC16-CCITT-FALSE covers the first 16 bytes. ops: 0=disarm, 1=arm zero,
% 2=run. After timeout/fault, disarm then explicit zero-duty arm is required.
% Stop immediately disables the bridge; running duty slews at 50%/second.
% This is a forward-only bench PWM controller, not encoder speed regulation.
%#codegen
persistent s
if isempty(s)
    s = struct('bytes',zeros(1,18,'uint8'),'session',uint32(0), ...
        'seq',-1,'armed',false,'permit',false,'remaining',0, ...
        'target',0,'actual',0,'err',uint8(0));
end
if s.armed
    s.remaining = s.remaining - 1;
    if s.remaining <= 0
        s = halt(s, uint8(2));
    end
end

% Bit 0 is merely "no data"; any other UART error is a stop condition.
if bitand(uint16(rxStatus),uint16(65534)) ~= 0
    s = halt(s,uint8(8));
elseif rxLength == 1 && rxStatus == 0
    s.bytes(1:17) = s.bytes(2:18);
    s.bytes(18) = uint8(rx(1));
    if s.bytes(1) == uint8(195) && s.bytes(2) == uint8(60)
        supplied = uint16(s.bytes(17)) + bitshift(uint16(s.bytes(18)),8);
        if motor_crc16(s.bytes(1:16)) ~= supplied
            s = halt(s,uint8(3));
        else
            version = s.bytes(3);
            op = s.bytes(4);
            session = littleU32(s.bytes(5:8));
            seq = double(littleU32(s.bytes(9:12)));
            requested = double(s.bytes(13)) + 256*double(s.bytes(14));
            ttl = double(s.bytes(15)) + 256*double(s.bytes(16));
            if version ~= 1 || session == 0 || op > 2 || requested > 500 || ...
                    ttl < 50 || ttl > 250 || (op < 2 && requested ~= 0)
                s = halt(s,uint8(4));
            elseif op == 0 && session ~= s.session
                % Only a complete disarmed-zero frame may adopt a new host.
                s = halt(s,uint8(0));
                s.session = session;
                s.seq = seq;
                s.permit = true;
            elseif session ~= s.session
                s = halt(s,uint8(6));
            elseif seq <= s.seq
                s = halt(s,uint8(5));
            else
                s.seq = seq;
                if op == 0
                    s = halt(s,uint8(0));
                    s.permit = true;
                elseif op == 1
                    if s.permit || s.armed
                        s.armed = true;
                        s.permit = false;
                        s.target = 0;
                        s.actual = 0;
                        s.remaining = ttl;
                        s.err = uint8(0);
                    else
                        s = halt(s,uint8(7));
                    end
                elseif s.armed
                    s.target = requested/10;
                    s.remaining = ttl;
                    s.err = uint8(0);
                else
                    s = halt(s,uint8(7));
                end
            end
        end
        % A complete candidate has been consumed. Do not interpret a magic
        % sequence inside its payload as the start of another command.
        s.bytes(:) = uint8(0);
    end
end

if s.armed && s.remaining > 0
    s.actual = s.actual + min(0.05,max(-0.05,s.target-s.actual));
else
    s.actual = 0;
end
duty = s.actual;
enable = s.armed && s.remaining > 0;
armed = enable;
commandError = s.err;
end

function s = halt(s,reason)
s.armed = false;
s.permit = false;
s.remaining = 0;
s.target = 0;
s.actual = 0;
s.err = reason;
end

function value = littleU32(bytes)
value = uint32(bytes(1)) + bitshift(uint32(bytes(2)),8) + ...
    bitshift(uint32(bytes(3)),16) + bitshift(uint32(bytes(4)),24);
end
