import math
import numpy as np
import tkinter as tk
import serial
import struct

class IRobotMotion:
    def open(self):
        pass
    def close(self):
        pass
    def move(self, x_speed, y_speed, rot_speed):
        pass

class OmniMotionRobot(IRobotMotion):

    def __init__(self):
        self.serialObj = serial.Serial()

    def open(self, serial_port):
        self.serialObj.port = serial_port
        self.serialObj.open()

    def move(self, x_speed, y_speed, rot_speed):#////////////////////////
        robotSpeed = math.sqrt(x_speed * x_speed + y_speed * y_speed)
        robotDirectionAngle = math.atan2(y_speed, x_speed)
        wheelSpeedToMainboardUnits = 90.9
        wheelLinearVelocities = []
        for angle in [0,120,240]:
            wheelLinearVelocities.append(robotSpeed * math.cos(robotDirectionAngle - angle*math.pi/180) + 0.2 * rot_speed)
        
        wheelAngularSpeedMainboardUnits = [int(wheelLinearVelocity * wheelSpeedToMainboardUnits) for wheelLinearVelocity in wheelLinearVelocities]

        print(wheelAngularSpeedMainboardUnits) 
        baidid = struct.pack('<hhhHBH',0,20,-20,0,0,0xAAAA) # praegu constant v22rtused
        self.serialObj.write(baidid)

    def close(self):
        self.serialObj.close()

