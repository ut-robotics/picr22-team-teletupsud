#import serial
#import time
#import struct
#import serial.tools.list_ports

import serial
import time
import struct
serial_port = "COM6"
ser = serial.Serial(serial_port)

speeds = [0, 0, 0]
speed_step = 50
speed_dir = 1
speed_min = -100
speed_max = 100

thrower_speed = 3100



def feedback1():
    recieved = ser.read(size=8)
    s1, s2, s3, delimiter = struct.unpack('<hhhH', recieved)

    return s1, s2, s3, delimiter

aeg1 = time.time()

while True:
    baidid = struct.pack('<hhhHH',0, 0, 0, thrower_speed,0xAAAA)
    ser.write(baidid)  
    print(thrower_speed)  
    print(feedback1())
    if thrower_speed >= 7000: #Testing to find the perfect value for 2 beeps
        thrower_speed = 7000
    else:
        thrower_speed -= 100
    for i, speed in enumerate(speeds):
        speeds[i] += speed_step * speed_dir

    if speeds[0] >= speed_max:
        speed_dir = -1
    elif speeds[0] <= speed_min:
        speed_dir = 1


    time.sleep(1)

baidid = struct.pack('<hhhHBH', -10, 10, 10, thrower_speed, , 0xAAAA)
ser.write(baidid)

ser.close()