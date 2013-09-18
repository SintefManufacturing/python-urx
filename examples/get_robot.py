import urx
from IPython import embed

if __name__ == "__main__":
    try:
        robot = urx.Robot("192.168.1.6")
        r = robot
        print("Robot object is available as robot or r")
        embed()
    finally:
        robot.shutdown()
