#!/usr/bin/env python
from urx import Robot
import math3d
from math import pi
import logging

if __name__ == "__main__":
    try:
        robot = Robot( 'localhost')#, logLevel=logging.DEBUG, parserLogLevel=logging.DEBUG)
        r = robot
        from IPython.frontend.terminal.embed import InteractiveShellEmbed
        ipshell = InteractiveShellEmbed( banner1="\nStarting IPython shell, robot object is available\n")
        ipshell(local_ns=locals())
    finally:
        if "robot" in dir():
            robot.cleanup()

