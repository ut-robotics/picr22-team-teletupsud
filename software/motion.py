import math
import numpy as np
import tkinter as tk
import serial
import struct
import serial.tools.list_ports
import time as time

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
                self.device = x.device

        self.serialObj = serial.Serial()

        self.wheel_distance_from_center = 0.2
        self.wheelSpeedToMainboardUnits = 90.9

    def open(self):
        self.serialObj.port = self.device
        self.serialObj.open()

    def move(self, x_speed, y_speed, rot_speed):#////////////////////////
        robotSpeed = math.sqrt(x_speed * x_speed + y_speed * y_speed)
        robotDirectionAngle = math.atan2(y_speed, x_speed)

        wheelLinearVelocities = []
        for angle in [0,120,240]:
            wheelLinearVelocities.append(robotSpeed * math.cos(robotDirectionAngle - math.radians(angle)) + self.wheel_distance_from_center * rot_speed)
        
        wheelAngularSpeedMainboardUnits = []
        for wheelLinearVelocity in wheelLinearVelocities:
            wheelLinearVelocity = wheelLinearVelocity * self.wheelSpeedToMainboardUnits
            if -1 < wheelLinearVelocity < 0:
                wheelLinearVelocity = -1
            elif 0 < wheelLinearVelocity < 1: 
                wheelLinearVelocity = 1
            wheelAngularSpeedMainboardUnits.append(int(wheelLinearVelocity))

        print(wheelAngularSpeedMainboardUnits) #                                                                                   Thrower_speed/Fail safe
        bytes_struct = struct.pack('<hhhHBH',wheelAngularSpeedMainboardUnits[1],wheelAngularSpeedMainboardUnits[0],wheelAngularSpeedMainboardUnits[2],0,0,0xAAAA)
        self.serialObj.write(bytes_struct)

    def throw(self,thrower_speed):
        bytes_struct = struct.pack('<hhhHBH', 20, 0, -20, thrower_speed,0,0xAAAA)
        self.serialObj.write(bytes_struct)
    def rotate(self):
        bytes_struct = struct.pack('<hhhHBH', 0, 10, 0, 0,0,0xAAAA)
        self.serialObj.write(bytes_struct)

    def close(self):
        self.serialObj.close()

