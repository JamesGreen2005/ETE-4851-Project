function text = pi_motor_text(h,r,p,cal,valid,imuError,armed,duty,commandError)
% Add actual motor state to the existing readable IMU telemetry.
%#codegen
base = bno055_text(h,r,p,cal,valid,imuError);
tail = uint8(' ARM=0 DUTY=00.0 CMDERR=000');
tail(6) = uint8(48) + uint8(armed);
v = min(500,round(max(0,duty)*10));
tail(13) = uint8(48+floor(v/100));
tail(14) = uint8(48+mod(floor(v/10),10));
tail(16) = uint8(48+mod(v,10));
e = double(commandError);
tail(25) = uint8(48+floor(e/100));
tail(26) = uint8(48+mod(floor(e/10),10));
tail(27) = uint8(48+mod(e,10));
text = [base(1:54),tail,uint8([13 10])];
end
