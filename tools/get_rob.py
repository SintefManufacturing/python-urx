#!/usr/bin/env python
import sys
import logging
from math import pi
from IPython import embed

from urx import Robot
import math3d

if __name__ == "__main__":
    if len(sys.argv) > 1:
        host = sys.argv[1]
    else:
        host = 'localhost'
    try:
        robot = Robot(host)
        r = robot
        embed()
    finally:
        if "robot" in dir():
            robot.close()

