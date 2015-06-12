"""
Python library to control an UR robot through its TCP/IP interface
"""
__version__ = "0.9.0"


from urx.urrobot import RobotException, URRobot

try:
    from urx.robot import Robot
except ImportError as ex:
    print("Exception while importing math3d base robot, disabling use of matrices", ex)
    Robot = URRobot
