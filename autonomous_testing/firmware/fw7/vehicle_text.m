% Diagnostic FW=7 BIN also inserts P2/A2/B2/X2 using generated C; see Vehicle_Drive_generated.c.
function text = vehicle_text(h,r,p,cal,valid,imuError,armed,duty,commandError,e1,e2,s1,s2,period)
%#codegen
base=bno055_text(h,r,p,cal,valid,imuError);
tail=uint8(' E1=00000 E2=00000 ARM=0 D1=+000.0 D2=+000.0 CMDERR=000');
tail(5:9)=digits(double(e1),5);
tail(14:18)=digits(double(e2),5);
tail(24)=uint8(48)+uint8(armed);
v=min(1000,round(abs(duty)*10));
if duty(1)<0, tail(29)=uint8('-'); end
if duty(2)<0, tail(39)=uint8('-'); end
tail(30:32)=digits(floor(v(1)/10),3); tail(34)=uint8(48+mod(v(1),10));
tail(40:42)=digits(floor(v(2)/10),3); tail(44)=uint8(48+mod(v(2),10));
tail(53:55)=digits(double(commandError),3);
diag=[uint8(' DRIVE=1 FW=7 S1='),digits(double(s1),4), ...
    uint8(' S2='),digits(double(s2),4),uint8(' T2='),digits(double(period),5)];
sh1=int16(0); sh2=int16(0); bv1=uint16(0); bv2=uint16(0); st1=uint8(0); st2=uint8(0);
if ~coder.target('MATLAB') && ~coder.target('Sfun')
    coder.cinclude('current_ina219.h');
    coder.ceval('current_ina219_status',coder.wref(sh1),coder.wref(bv1),coder.wref(st1),coder.wref(sh2),coder.wref(bv2),coder.wref(st2));
end
sensors=[uint8(' INA=1 SH1='),signed(sh1),uint8(' BV1='),digits(double(bv1),5),uint8(' ST1='),digits(double(st1),1), ...
    uint8(' SH2='),signed(sh2),uint8(' BV2='),digits(double(bv2),5),uint8(' ST2='),digits(double(st2),1)];
text=[base(1:54),diag,sensors,tail,uint8([13 10])];
end
function out=digits(v,n)
out=zeros(1,n,'uint8');
for k=n:-1:1
    out(k)=uint8(48+mod(v,10)); v=floor(v/10);
end
end

function out=signed(v)
sign=uint8('+'); if v<0, sign=uint8('-'); end
out=[sign,digits(abs(double(v)),5)];
end
