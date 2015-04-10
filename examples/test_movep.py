import time 
import urx
import logging


if __name__ == "__main__":
    rob = urx.Robot("192.168.1.6")
    try:
        l = 0.1
        v = 0.07
        a = 0.1
        r = 0.05
        pose = rob.getl()
        pose[2] += l
        rob.movep(pose, acc=a, vel=v, radius=r, wait=False)
        while True:
            p = rob.getl(wait=True)
            if p[2] > pose[2] - 0.05:
                break

        pose[1] += l 
        rob.movep(pose, acc=a, vel=v, radius=r, wait=False)
        while True:
            p = rob.getl(wait=True)
            if p[1] > pose[1] - 0.05:
                break

        pose[2] -= l
        rob.movep(pose, acc=a, vel=v, radius=r, wait=False)
        while True:
            p = rob.getl(wait=True)
            if p[2] < pose[2] + 0.05:
                break

        pose[1] -= l
        rob.movep(pose, acc=a, vel=v, radius=0, wait=True)

    finally:
        rob.close()

