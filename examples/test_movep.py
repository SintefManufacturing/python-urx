import urx
import logging

if __name__ == "__main__":
    rob = urx.Robot("192.168.128.120", logLevel=logging.INFO)
    try:
        l = 0.05
        v = 0.05
        a = 0.3
        r = 0.005
        pose = rob.getl()
        pose[2] += l
        rob.movep(pose, acc=a, vel=v, radius=r, wait=False)
        pose[1] += l 
        rob.movep(pose, acc=a, vel=v, radius=r, wait=False)
        pose[2] -= l
        rob.movep(pose, acc=a, vel=v, radius=r, wait=False)
        pose[1] -= l
        rob.movep(pose, acc=a, vel=v, radius=0, wait=False)

    finally:
        rob.cleanup()

