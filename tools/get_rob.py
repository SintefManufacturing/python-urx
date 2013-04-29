#!/usr/bin/env python
import sys
import logging
from math import pi

from urx import Robot
import math3d

if __name__ == "__main__":
    if len(sys.argv) > 1:
        host = sys.argv[1]
    else:
        host = 'localhost'
    try:
        robot = Robot( host )#, logLevel=logging.DEBUG, parserLogLevel=logging.DEBUG)
        r = robot
        from IPython.frontend.terminal.embed import InteractiveShellEmbed
        ipshell = InteractiveShellEmbed( banner1="\nStarting IPython shell, robot object is available\n")
        ipshell(local_ns=locals())
    finally:
        if "robot" in dir():
            robot.cleanup()

