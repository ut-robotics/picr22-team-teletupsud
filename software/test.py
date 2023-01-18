#import serial
#import time
#import struct
#import serial.tools.list_ports
#myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
#print (myports)

import serial
import time
import struct
serial_port = "/dev/ttyACM0"

ser = serial.Serial(serial_port)

disable_failsafe = 0
speed1 = 0
speed2 = 0
speed3 = 0

baidid = struct.pack('<hhhHH', 0, 0, 0,2900,  0xAAAA)
baidid2 = struct.pack('<hhhHH', 0, 0, 0,4000,  0xAAAA)
ser.write(baidid)

aeg1 = time.time()
while True:
    ser.write(baidid)
    aeg2 = time.time()
    if aeg2-aeg1 > 4:
        break
ser.write(baidid2)

ser.close()