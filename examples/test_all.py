"""
Testing script that runs many of the urx methods, while attempting to keep robot pose around its starting pose
"""

from math import pi
import time

import urx
import logging

if __name__ == "__main__":
    rob = urx.Robot("192.168.1.6", logLevel=logging.WARNING)
    rob.set_tcp((0, 0, 0, 0, 0, 0))
    rob.set_payload(0.5, (0, 0, 0))
    try:
        l = 0.05
        v = 0.05
        a = 0.3
        r = 0.01
        print("Digital out 0 and 1 are: ", rob.get_digital_out(0), rob.get_digital_out(1) )
        print("Analog inputs are: ", rob.get_analog_inputs())
        j = rob.getj()
        print("Initial joint configuration is ", j)
        t = rob.get_transform()
        print("Transformation from base to tcp is: ", t)
        print("Translating in x")
        rob.translate((l, 0, 0), acc=a, vel=v)
        pose = rob.getl()
        print("robot tcp is at: ", pose)
        print("moving in z")
        pose[2] += l
        rob.movel(pose, acc=a, vel=v, wait=False)
        print("Waiting for end move")
        rob.wait_for_move()
        #FIXME add movep if controller version newer than XX
        print("Moving through several points with a radius")
        pose[0] -= l
        p1 = pose[:]
        pose[2] -= l
        p2 = pose[:]
        rob.movels([p1, p2], vel=v, acc=a, radius=r)

        print("rotate tcp around around base z ")
        t.orient.rotate_zb(pi / 8)
        rob.apply_transform(t, vel=v, acc=a)

        print("moving in tool z")
        rob.translate_tool((0, 0, l), vel=v, acc=a)

        print("moving in tool -z using speed command")
        rob.speedl_tool((0, 0, -v, 0, 0, 0), acc=a, min_time=3)
        print("Waiting 2 seconds2")
        time.sleep(2)
        print("stop robot")
        rob.stopj()

        print("Sending robot back to original position")
        rob.movej(j, acc=0.8, vel=0.2) 


    finally:
        rob.cleanup()

