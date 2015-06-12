from math import pi

import urx
import logging

if __name__ == "__main__":
    rob = urx.Robot("192.168.1.100")
    #rob = urx.Robot("localhost")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    try:
        l = 0.05
        v = 0.05
        a = 0.3
        j = rob.getj()
        print("Initial joint configuration is ", j)
        t = rob.get_pose()
        print("Transformation from base to tcp is: ", t)
        print("Translating in x")
        rob.translate((l, 0, 0), acc=a, vel=v)
        pose = rob.getl()
        print("robot tcp is at: ", pose)
        print("moving in z")
        pose[2] += l
        rob.movel(pose, acc=a, vel=v)


        print("Translate in -x and rotate")
        t.orient.rotate_zb(pi/4)
        t.pos[0] -= l
        rob.set_pose(t, vel=v, acc=a)
        print("Sending robot back to original position")
        rob.movej(j, acc=0.8, vel=0.2) 


    finally:
        rob.close()

