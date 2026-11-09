function crc = vehicle_crc16(bytes)
% CRC16-CCITT-FALSE: polynomial 0x1021, init FFFF, no reflection/xorout.
%#codegen
crc = uint16(65535);
for k=1:numel(bytes)
    crc = bitxor(crc,bitshift(uint16(bytes(k)),8));
    for bit=1:8
        high = bitand(crc,uint16(32768)) ~= 0;
        crc = bitshift(crc,1);
        if high
            crc = bitxor(crc,uint16(4129));
        end
    end
end
end
