"""
example program to control a universal robot with a joystick
All joysticks are differens, so you will need to modify the script to work with your joystick
"""

import time

import pygame

import math3d as m3d
from math import pi

import urx

class Cmd(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.axis0 = 0
        self.axis1 = 0
        self.axis2 = 0 
        self.axis3 = 0
        self.axis4 = 0
        self.axis5 = 0 
        self.btn0 = 0
        self.btn1 = 0
        self.btn2 = 0
        self.btn3 = 0
        self.btn4 = 0
        self.btn5 = 0
        self.btn6 = 0
        self.btn7 = 0
        self.btn8 = 0
        self.btn9 = 0


class Service(object):
    def __init__(self, robot, linear_velocity=0.1, rotational_velocity=0.1, acceleration=0.1):
        self.joystick = None
        self.robot = robot
        #max velocity and acceleration to be send to robot
        self.linear_velocity = linear_velocity
        self.rotational_velocity = rotational_velocity
        self.acceleration = acceleration
        #one button send the robot to a preprogram position defined by this variable in join space
        self.init_pose = [-2.0782002408411593, -1.6628931459654561, 2.067930303382134, -1.9172217394630149, 1.5489023943220621, 0.6783171005488982]
        
        self.cmd = Cmd()

    def init_joystick(self):
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print('Initialized Joystick : %s' % self.joystick.get_name())

    def loop(self):
        print("Starting loop")
        air = False
        while True:
            self.cmd.reset()
            pygame.event.pump()#Seems we need polling in pygame...
            
            #get joystick state
            for i in range(0, self.joystick.get_numaxes()):
                val = self.joystick.get_axis(i)
                if i in (2, 5) and val != 0:
                    val += 1
                if abs(val) < 0.2:
                    val = 0
                tmp = "self.cmd.axis" + str(i) + " = " + str(val)
                if val != 0:
                    print(tmp)
                exec(tmp)
            
            #get button state
            for i in range(0, self.joystick.get_numbuttons()):
                if self.joystick.get_button(i) != 0:
                    tmp = "self.cmd.btn" + str(i) + " = 1"
                    print(tmp)
                    exec(tmp)

            if self.cmd.btn0:
                #toggle IO
                air = not air
                self.robot.set_digital_out(2, air)

            if self.cmd.btn9:
                print("moving to init pose")
                self.robot.movej(self.init_pose, acc=1, vel=0.1)
            
            #initalize speed array to 0
            speeds = [0, 0, 0, 0, 0, 0]

            #get linear speed from joystick
            speeds[0] = -1 * self.cmd.axis0 * self.linear_velocity
            speeds[1] = self.cmd.axis1 * self.linear_velocity
            if self.cmd.btn4 and not self.cmd.axis2:
                speeds[2] = -self.linear_velocity
            if self.cmd.axis2 and not self.cmd.btn4:
                speeds[2] = self.cmd.axis2 * self.linear_velocity

            #get rotational speed from joystick
            speeds[4] = -1 * self.cmd.axis3 * self.rotational_velocity
            speeds[3] = -1 * self.cmd.axis4 * self.rotational_velocity
            if self.cmd.btn5 and not self.cmd.axis5:
                speeds[5] = self.rotational_velocity
            if self.cmd.axis5 and not self.cmd.btn5:
                speeds[5] = self.cmd.axis5 * -self.rotational_velocity
            
            #for some reasons everything is inversed
            speeds = [-i for i in speeds]
            #Now sending to robot. tol by default and base csys if btn2 is on
            if speeds != [0 for _ in speeds]:
                print("Sending ", speeds)
            if self.cmd.btn7:
                self.robot.speedl_tool(speeds, acc=self.acceleration, min_time=2)
            else:
                self.robot.speedl(speeds, acc=self.acceleration, min_time=2)


    def close(self):
        if self.joystick:
            self.joystick.quit()


if __name__ == "__main__":
    robot = urx.Robot("192.168.1.100")
    r = robot


    #start joystick service with given max speed and acceleration
    service = Service(robot, linear_velocity=0.1, rotational_velocity=0.1, acceleration=0.1)
    service.init_joystick()
    try:
        service.loop() 
    finally: 
        robot.close()
        service.close()
