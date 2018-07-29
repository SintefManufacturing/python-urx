"""
Python library to control an UR robot through its TCP/IP interface
"""
from urx.urrobot import RobotException, URRobot  # noqa

__version__ = "0.11.0"

try:
    from urx.robot import Robot
except ImportError as ex:
    print("Exception while importing math3d base robot, disabling use of matrices", ex)
    Robot = URRobot
