function line = bno055_text(heading, roll, pitch, calibration, valid, errorCode)
% Fixed 56-byte ASCII telemetry, degrees, SYS/GYR/ACC/MAG calibration 0..3.
% VALID means a complete orientation frame less than 300 ms old, not calibrated.
%#codegen
line = [uint8('H=+000.0 R=+000.0 P=+000.0 CAL=0/0/0/0 VALID=0 ERR=000'),uint8([13 10])];
line(3:8) = angleText(heading);
line(12:17) = angleText(roll);
line(21:26) = angleText(pitch);
line(32) = uint8(48) + bitand(bitshift(uint8(calibration),-6),uint8(3));
line(34) = uint8(48) + bitand(bitshift(uint8(calibration),-4),uint8(3));
line(36) = uint8(48) + bitand(bitshift(uint8(calibration),-2),uint8(3));
line(38) = uint8(48) + bitand(uint8(calibration),uint8(3));
line(46) = uint8(48) + uint8(valid);
e = double(errorCode);
line(52) = uint8(48 + floor(e/100));
line(53) = uint8(48 + mod(floor(e/10),10));
line(54) = uint8(48 + mod(e,10));
end

function text = angleText(value)
text = uint8('+000.0');
if value < 0
    text(1) = uint8('-');
end
v = min(9999, round(abs(value)*10));
text(2) = uint8(48 + floor(v/1000));
text(3) = uint8(48 + mod(floor(v/100),10));
text(4) = uint8(48 + mod(floor(v/10),10));
text(6) = uint8(48 + mod(v,10));
end
