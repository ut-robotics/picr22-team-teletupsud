import math
import numpy as np
import tkinter as tk
import serial
import struct
import serial.tools.list_ports

class IRobotMotion:
    def open(self):
        pass
    def close(self):
        pass
    def move(self, x_speed, y_speed, rot_speed):
        pass

class OmniMotionRobot(IRobotMotion):

    def __init__(self):
        pid = 22336
        for x in serial.tools.list_ports.comports():
            if x.pid == pid:
                self.serialObj = serial.Serial(x.device)
            else:
                self.serialObj = serial.Serial()

            self.wheel_distance_from_center = 0.2
            self.wheelSpeedToMainboardUnits = 90.9

    def open(self):
        pass

    def move(self, x_speed, y_speed, rot_speed):#////////////////////////
        robotSpeed = math.sqrt(x_speed * x_speed + y_speed * y_speed)
        robotDirectionAngle = math.atan2(y_speed, x_speed)

        wheelLinearVelocities = []
        for angle in [0,120,240]:
            wheelLinearVelocities.append(robotSpeed * math.cos(robotDirectionAngle - angle*math.pi/180) + self.wheel_distance_from_center * rot_speed)
        
        wheelAngularSpeedMainboardUnits = [int(wheelLinearVelocity * self.wheelSpeedToMainboardUnits) for wheelLinearVelocity in wheelLinearVelocities]

        print(wheelAngularSpeedMainboardUnits) #                                                                                   Thrower_speed/Fail safe
        baidid = struct.pack('<hhhHBH',wheelAngularSpeedMainboardUnits[1],wheelAngularSpeedMainboardUnits[0],wheelAngularSpeedMainboardUnits[2],0,0,0xAAAA)
        self.serialObj.write(baidid)

    def close(self):
        self.serialObj.close()

