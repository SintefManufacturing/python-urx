from math import pi

import urx
import logging

if __name__ == "__main__":
    rob = urx.Robot("192.168.128.120", logLevel=logging.INFO)
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    try:
        l = 0.05
        v = 0.05
        a = 0.3
        rob.translate((l, 0, 0))
        pose = rob.getl()
        print("robot tcp is at: ", pose)
        pose[2] += l
        rob.movel(pose, acc=a, vel=v)


        t = rob.get_transform()
        print("Transformation from base to tcp is: ", t)
        t.orient.rotate_zb(pi/4)
        t.pos[0] -= l
        rob.apply_transform(t, vel=v, acc=a)
        t.pos[2] -= l
        new_t = rob.apply_transform(t, vel=v, acc=a)
        print("Transformation from base to tcp is: ", new_t)


    finally:
        rob.cleanup()

