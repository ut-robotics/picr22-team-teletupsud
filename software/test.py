import serial
import time
import struct
serial_port = "/dev/ttyACM0"

ser = serial.Serial(serial_port)

disable_failsafe = 0
speed1 = 0
speed2 = 0
speed3 = 0
thrower_speed = 400

baidid = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)
#ser.write(baidid)

aeg1 = time.time()
while True:
    ser.write(baidid)
    aeg2 = time.time()
    if aeg2-aeg1 > 40:
        print(aeg2)
        break
print("vsjo")
#time.sleep(4)

baidid = struct.pack('<hhhHBH', 0, 0, 0, thrower_speed, disable_failsafe, 0xAAAA)
ser.write(baidid)

ser.close()