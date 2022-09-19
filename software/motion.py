import turtle
import math
import numpy as np
import time
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
    #//////////////////////// LIIGUTA SIIA MOVE ALT
    serial_port = "/dev/ttyACM0"

    def __init__(self):
        self.serialObj = serial.Serial()

    def open(self, serial_port):
        self.serialObj.port = serial_port
        self.serialObj.open()

    def move(self, x_speed, y_speed, rot_speed):#////////////////////////
        speeds = [0, 0, 0]
        x_speed, y_speed, rot_speed = 0, 20, 0
        robotSpeed = math.sqrt(x_speed * x_speed + y_speed * y_speed)
        robotDirectionAngle = math.atan2(y_speed, x_speed)
        print(robotDirectionAngle)
        wheelSpeedToMainboardUnits = 90.9
        wheelLinearVelocities = []
        for angle in [0,120,240]:
            wheelLinearVelocities.append(int(robotSpeed * math.cos(robotDirectionAngle - angle*math.pi/180) + 0.2 * rot_speed))
        
        wheelAngularSpeedMainboardUnits = [wheelLinearVelocity * wheelSpeedToMainboardUnits for wheelLinearVelocity in wheelLinearVelocities] 
        baidid = struct.pack('<hhhHBH',0,0,0,0,0,0xAAAA)
        self.serialObj.write(baidid)

    def close(self):
        self.serialObj.close()


class TurtleRobot(IRobotMotion):
    def __init__(self, name="Default turtle robot"):

        window = tk.Tk()
        window.title(name)

        canvas = tk.Canvas(master=window, width=500, height=500)
        canvas.pack()

        self.screen = turtle.TurtleScreen(canvas)
        self.turtle_obj = turtle.RawTurtle(self.screen)
        self.turtle_obj.speed('fastest')

        self.steps = 20

    def open(self):
        print("Wroom! Starting up turtle!")

    def close(self):
        print("Going to dissapear...")

    #Very dumb logic to draw motion using turtle
    def move(self, x_speed, y_speed, rot_speed):

                
        self.screen.tracer(0, 0)
        angle_deg = 0

        angle_deg = np.degrees(math.atan2(x_speed, y_speed))

        distance = math.sqrt(math.pow(x_speed, 2) + math.pow(y_speed, 2))

        distance_step = distance / float(self.steps)
        angel_step = np.degrees(rot_speed / float(self.steps))

        self.turtle_obj.penup()
        self.turtle_obj.reset()
        self.turtle_obj.right(angle_deg - 90)
        self.turtle_obj.pendown()

        for i in range(0, self.steps):
            self.turtle_obj.right(angel_step)
            self.turtle_obj.forward(distance_step)

        self.turtle_obj.penup()
        self.screen.update()


class TurtleOmniRobot(TurtleRobot):
    def __init__(self, name="Default turtle omni robot"):
        TurtleRobot.__init__(self, name)

        # Wheel angles
        self.motor_config = [0, 120, 240]

    def move(self, x_speed, y_speed, rot_speed):#////////////////////////
        speeds = [0, 0, 0]
        x_speed, y_speed, rot_speed = 0, 20, 0
        robotSpeed = math.sqrt(x_speed * x_speed + y_speed * y_speed)
        robotDirectionAngle = math.atan2(y_speed, x_speed)
        print(robotDirectionAngle)
        wheelSpeedToMainboardUnits = 90.9
        wheelLinearVelocities = []
        for angle in self.motor_config:
            wheelLinearVelocities.append(int(robotSpeed * math.cos(robotDirectionAngle - angle*math.pi/180) + 0.2 * rot_speed))
        
        wheelAngularSpeedMainboardUnits = [wheelLinearVelocity * wheelSpeedToMainboardUnits for wheelLinearVelocity in wheelLinearVelocities] 


        TurtleRobot.move(self, x_speed, y_speed, rot_speed)

    def speeds_to_direction(self, speeds):
        offset_x = 0
        offset_y = 0
        degree = int((speeds[0] + speeds[1] + speeds[2]) / 3)
    
        for i in range(0, 3):
            end_vector = self.motor_side_forward_scale(self.motor_config[i] + 90, speeds[i], offset_x, offset_y)
            offset_x = end_vector[0]
            offset_y = end_vector[1]
    
        offsets = [offset_x * -1, offset_y]
        speeds = [int(a / 1.5) for a in offsets]
        speeds.append(degree)
    
        return speeds
 
    def motor_side_forward_scale(self, angel, length, offset_x=0, offset_y=0):
        ang_rad = math.radians(angel)
        return [length * math.cos(ang_rad) + offset_x, length * math.sin(ang_rad) + offset_y]
