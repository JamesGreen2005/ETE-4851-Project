function [duty, enable, armed, commandError, servoPulse] = vehicle_command(rx, rxLength, rxStatus)
% Protocol v5: 24 bytes, C3 3C/version/op/session/sequence, two signed
% 0.1% duties, two uint16 servo pulses, TTL, CRC16 over first 22 bytes.
% Boot/stop/fault: bridges disabled, servo signals off. Arm at zero duty.
% TIM2 steering 1000..2000 us; motor duties limited to +/-100%.
%#codegen
persistent s
if isempty(s)
    s = struct('bytes',zeros(1,24,'uint8'),'session',uint32(0), ...
        'seq',-1,'armed',false,'permit',false,'remaining',0, ...
        'target',zeros(1,2),'actual',zeros(1,2),'err',uint8(0), ...
        'lastSign',zeros(1,2),'zeroTicks',500*ones(1,2), ...
        'servo',zeros(1,2));
end
if s.armed || any(s.servo ~= 0)
    s.remaining = s.remaining - 1;
    if s.remaining <= 0
        s = halt(s, uint8(2));
    end
end

% Bit 0 is merely "no data"; any other UART error is a stop condition.
if bitand(uint16(rxStatus),uint16(65534)) ~= 0
    s = halt(s,uint8(8));
elseif rxLength == 1 && rxStatus == 0
    s.bytes(1:23) = s.bytes(2:24);
    s.bytes(24) = uint8(rx(1));
    if s.bytes(1) == uint8(195) && s.bytes(2) == uint8(60)
        supplied = uint16(s.bytes(23)) + bitshift(uint16(s.bytes(24)),8);
        if vehicle_crc16(s.bytes(1:22)) ~= supplied
            s = halt(s,uint8(3));
        else
            version = s.bytes(3);
            op = s.bytes(4);
            session = littleU32(s.bytes(5:8));
            seq = double(littleU32(s.bytes(9:12)));
            requested = [double(s.bytes(13)) + 256*double(s.bytes(14)), double(s.bytes(15)) + 256*double(s.bytes(16))];
            for motor=1:2
                if requested(motor)>=32768, requested(motor)=requested(motor)-65536; end
            end
            pulses = [double(s.bytes(17))+256*double(s.bytes(18)), double(s.bytes(19))+256*double(s.bytes(20))];
            ttl = double(s.bytes(21)) + 256*double(s.bytes(22));
            servoValid = all(pulses >= 1000 & pulses <= 2000);
            if version ~= 5 || session == 0 || op > 2 || any(abs(requested)>1000) || ...
                    ttl < 50 || ttl > 250 || (op < 2 && any(requested ~= 0)) || ...
                    (op == 0 && any(pulses ~= 0)) || (op ~= 0 && ~servoValid)
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
                        s.servo = pulses;
                        s.target = zeros(1,2);
                        s.actual = zeros(1,2);
                        s.remaining = ttl;
                        s.err = uint8(0);
                    else
                        s = halt(s,uint8(7));
                    end
                elseif s.armed
                    s.servo = pulses;
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

for motor=1:2
    desired=0;
    if s.armed && s.remaining>0, desired=s.target(motor); end
    % Ramp to zero before reversal, then hold zero for at least 500 ms.
    if desired~=0 && s.lastSign(motor)~=0 && sign(desired)~=s.lastSign(motor)
        if s.actual(motor)~=0 || s.zeroTicks(motor)<500, desired=0; end
    end
    if s.armed && s.remaining>0
        s.actual(motor)=desired; % Immediate requested duty; reversal hold retained.
        if abs(s.actual(motor))<1e-9, s.actual(motor)=0; end
    else
        s.actual(motor)=0;
    end
    if s.actual(motor)==0
        s.zeroTicks(motor)=min(500,s.zeroTicks(motor)+1);
    else
        s.zeroTicks(motor)=0;
        s.lastSign(motor)=sign(s.actual(motor));
    end
end
duty = s.actual;
enable = s.armed && s.remaining > 0;
armed = enable;
commandError = s.err;
servoPulse = s.servo;
end

function s = halt(s,reason)
s.armed = false;
s.permit = false;
s.remaining = 0;
s.target = zeros(1,2);
s.actual = zeros(1,2);
s.servo = zeros(1,2);
s.err = reason;
end

function value = littleU32(bytes)
value = uint32(bytes(1)) + bitshift(uint32(bytes(2)),8) + ...
    bitshift(uint32(bytes(3)),16) + bitshift(uint32(bytes(4)),24);
end
