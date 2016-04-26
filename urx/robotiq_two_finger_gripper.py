"""
Python library to control Robotiq Two Finger Gripper connected to UR robot via Python-URX

Tested using a UR5 Version CB3 and Robotiq 2-Finger Gripper Version 85

SETUP

You must install the driver first (http://support.robotiq.com/pages/viewpage.action?pageId=5963876) and then power on the gripper from the gripper UI

FAQ

Why does this class group all the commands together and run them as a single program as opposed to running each line seperately (like most of URX)?

- The gripper is controlled by connecting to the robot's computer (TCP/IP) and then communicating with the gripper via a socket (127.0.0.1:63352).  The scope of the socket is at the program level.  It will be automatically closed whenever a program finishes.  Therefore it's important that we run all commands as a single program.

DOCUMENTATION

- This code was developed by downloading the "gripper package" on http://support.robotiq.com/pages/viewpage.action?pageId=5963876
- Open folder "robotiq_2f_gripper_programs_CB3"
- robotiq_2f_gripper_programs_CB3/advanced_template_test.script was referenced to create this class

Future Features

- Though I haven't developed it yet if you look in robotiq_2f_gripper_programs_CB3/advanced_template_test.script and view function "rq_get_var" there is an example of how to determine the current state of the gripper and if it's holding an object.
"""

import logging

class Robotiq_Two_Finger_Gripper(object):
	complete_program = ""
	header = "def myProg():" + b"\n"
	end =  b"\n" + "end"
	logger = False

	def __init__(self):
		self.logger = logging.getLogger("urx")
		self.reset()

	def reset(self):
		self.complete_program = ""
		self.add_line_to_program("set_analog_inputrange(0, 0)")
		self.add_line_to_program("set_analog_inputrange(1, 0)")
		self.add_line_to_program("set_analog_inputrange(2, 0)")
		self.add_line_to_program("set_analog_inputrange(3, 0)")
		self.add_line_to_program("set_analog_outputdomain(0, 0)")
		self.add_line_to_program("set_analog_outputdomain(1, 0)")
		self.add_line_to_program("set_tool_voltage(0)")
		self.add_line_to_program("set_runstate_outputs([])")
		self.add_line_to_program("set_payload(0.85)") #0.85 is the weight of the gripper in KG
		self.add_line_to_program("socket_close(\"gripper_socket\")")
		#self.add_line_to_program("sleep(1)") #in Robotiq's example they do a wait here... I haven't found it nec
		self.add_line_to_program("socket_open(\"127.0.0.1\",63352,\"gripper_socket\")")
		#self.add_line_to_program("sleep(1)")
		self.add_line_to_program("socket_set_var(\"SEP\",255,\"gripper_socket\")") #Speed 0-255 is valid
		self.add_line_to_program("sync()")
		self.add_line_to_program("socket_set_var(\"FOR\",50,\"gripper_socket\")") #Force 0-255 is valid
		self.add_line_to_program("sync()")
		self.add_line_to_program("socket_set_var(\"ACT\",1,\"gripper_socket\")") # Activate robot
		self.add_line_to_program("sync()")
		self.add_line_to_program("socket_set_var(\"GTO\",1,\"gripper_socket\")")
		self.add_line_to_program("sync()")

	def open_gripper(self):
		self.add_line_to_program("socket_set_var(\"POS\",0,\"gripper_socket\")") #0 is open; range is 0-255
		self.add_line_to_program("sync()")
		self.add_line_to_program("sleep(2)")

	def close_gripper(self):
		self.add_line_to_program("socket_set_var(\"POS\",255,\"gripper_socket\")") #255 is closed; range is 0-255
		self.add_line_to_program("sync()")
		self.add_line_to_program("sleep(2)")

	def add_line_to_program(self,new_line):
		if(self.complete_program != ""):
			self.complete_program += b"\n"
		self.complete_program += new_line

	def ret_program_to_run(self):
		if(self.complete_program == ""):
			self.logger.debug("robotiq_two_finger_gripper's program is empty")
			return ""

		prog = self.header
		prog += self.complete_program
		prog += self.end
		return prog