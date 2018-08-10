
from urx.urscript import URScript
import time
import os

class OnRobotGripperRG2Script(URScript):
    
    def __init__(self):
        super(OnRobotGripperRG2Script, self).__init__()
        print(os.listdir("."))
        with open("../urx/onrobot.script", "r") as f:
            for line in f.readlines():
                self.add_line_to_program(line)

    def _rg2_command(self, target_width=110, target_force=40, payload=0.5, set_payload=False, depth_compensation=False, slave=False):
        self.add_line_to_program("RG2(target_width={}, target_force={}, payload={}, set_payload={}, depth_compensation={}, slave={})"
                                 .format(target_width, target_force, payload, set_payload, depth_compensation, slave))
        # self.add_line_to_program("RG2(110)")

class OnRobotGripperRG2(object):

    def __init__(self, robot):
        self.robot = robot

    def open_gripper(self):
        urscript = OnRobotGripperRG2Script()
        urscript._rg2_command(target_width=110)
        print(urscript())
        self.robot.send_program(urscript())
        time.sleep(2)
    def close_gripper(self):
        urscript = OnRobotGripperRG2Script()
        urscript._rg2_command(target_width=0)
        print(urscript())
        self.robot.send_program(urscript())
        time.sleep(2)
