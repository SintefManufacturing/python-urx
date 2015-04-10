from __future__ import division

import spnav
import time
import math3d as m3d
from math import pi

import urx

class Cmd(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.rx = 0
        self.ry = 0
        self.rz = 0
        self.btn0 = 0
        self.btn1 = 0

    def get_speeds(self):
        return [self.x, self.y, self.z, self.rx, self.ry, self.rz]



class Service(object):
    def __init__(self, robot):
        self.robot = robot
        self.lin_coef = 5000
        self.rot_coef = 5000

    def loop(self):
        ts = 0 
        btn0_state = 0
        btn_event = None
        cmd = Cmd()
        while True:
            time.sleep(0.01)
            cmd.reset()
            #spnav.spnav_remove_events(spnav.SPNAV_EVENT_ANY) # seems broken
            event = spnav.spnav_poll_event()
            if event:
                if type(event) is spnav.SpnavButtonEvent:
                    btn_event = event
                    if event.bnum == 0:
                        btn0_state = event.press
                elif type(event) is spnav.SpnavMotionEvent:
                    if abs(event.translation[0]) > 30:
                        cmd.y = event.translation[0] / self.lin_coef
                    if abs(event.translation[1]) > 30:
                        cmd.z = -1 * event.translation[1] / self.lin_coef
                    if abs(event.translation[2]) > 30:
                        cmd.x = event.translation[2] / self.lin_coef
                    if abs(event.rotation[0]) > 20:
                        cmd.ry = event.rotation[0] / self.lin_coef
                    if abs(event.rotation[1]) > 20:
                        cmd.rz = -1 * event.rotation[1] / self.lin_coef
                    if abs(event.rotation[2]) > 20:
                        cmd.rx = event.rotation[2] / self.lin_coef
            
            if (time.time() - ts) > 0.12:
                ts = time.time()
                speeds = cmd.get_speeds()
                if btn0_state:
                    self.robot.speedl_tool(speeds, acc=0.10, min_time=2)
                else:
                    self.robot.speedl(speeds, acc=0.10, min_time=2)
                btn_event = None
                speeds = cmd.get_speeds()
                #if speeds != [0 for _ in speeds]:
                print(event)
                print("Sending", speeds)


if __name__ == '__main__':
    spnav.spnav_open()
    robot = urx.Robot("192.168.0.90")
    #robot = urx.Robot("localhost")
    robot.set_tcp((0, 0, 0.27, 0, 0, 0))
    trx = m3d.Transform()
    trx.orient.rotate_zb(pi/4)
    robot.set_csys("mycsys", trx)
    service = Service(robot)
    try:
        service.loop()
    finally:
        robot.close()
        spnav.spnav_close()



