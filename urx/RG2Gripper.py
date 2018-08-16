#! /usr/bin/env python
from urx.robot import Robot
import threading
import logging
import time

"""         Created by David Hinwood, University of Canberra
            This function opens and closes the RG2 gripper produced by on robot
            This work was inspired by Mark Silliman who produced a class to control
            a gripper made by robotiq in urx and Sharath Jotawar who created a ROS package
            used in moveit to control the RG2 gripper
            
            Notes:
                -Requires an Ethernet connection to your ur10, dynamically finds IP address
                -While an object is grasped, the gripper is still going to 0 meaning if 
                 you remove the object from the gripper while having set the width to zero
                 it will still go to zero
                -Possible ways to determine grasping include setting the width to max for
                 non grasping movements and to evaluate pickup check the width

	    
            Future feautes
	    While it is possible to currently infer whether the robot is currently holding 
	    something it would be a beneficial feature to read the force being applied and
	    return to user grasp success. Unfortunatly this was unfeasible during the first 
	    round at development. It is on the horizon but on an unknown timeframe.

	    License: LGPL-3.0
"""


class RG2:
    '''Initialization of class, requires user to be connected to robot via ethernet'''

    def __init__(self, RobotParameter):
        import netifaces as ni
        ni.ifaddresses('eth0')
        self.ip = ni.ifaddresses('eth0')[ni.AF_INET][0]['addr']
        self.Robot = (RobotParameter)
        self.GripperMonitorSocket = 50002
        self.MonitorThreadRunning = False
        self.currentwidth = 0.0
        self.logger = logging.getLogger("urx")

    '''Gets the current width between the gripper fingers in mm'''

    def getWidth(self):
        '''opens a thread in the background to listen to a message from the gripper'''

        def StartBackgroundListenerService():
            data = float(-1)
            while self.MonitorThreadRunning == True:
                try:
                    import socket
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    server_address = (self.ip, 50002)
                    sock.bind(server_address)
                    sock.listen(1)
                    connection, client_address = sock.accept()
                    data = connection.recv(64)
                    data = float(data)
                    self.currentwidth = data
                    connection.close()
                    self.MonitorThreadRunning = False
                except Exception as e:
                    print e
                    pass
            return

        cmd_str = "def rg2GripDetect():\n"
        cmd_str += "    zscale = (get_analog_in(2)-0.026)/2.976\n"
        cmd_str += "    zangle = zscale*1.57079633-0.087266462\n"
        cmd_str += "    zwidth = 5+110*sin(zangle)\n"
        cmd_str += "    global measure_width = (floor(zwidth*10))/10-0.0\n"
        cmd_str += "    textmsg(\"width\",measure_width)\n"
        cmd_str += "    socket_open(\"" + self.ip + "\", " + "50002" + ")\n"
        cmd_str += "        socket_send_string(measure_width)\n"
        cmd_str += "    socket_close()\n"
        cmd_str += "end\n"
        thread = threading.Thread(target=StartBackgroundListenerService, args=())
        thread.daemon = True
        self.MonitorThreadRunning = True
        thread.start()
        self.Robot.send_program(cmd_str)
	#small delay to wait for the thread
        time.sleep(0.2)
        return self.currentwidth


    '''Sets the width of the robot
    width can be anywhere between 0 and 110, required parameter
        force can be between 3 and 40, default is 20
    '''


    def setWidth(self, width, force=20):
        try:
            if width >= 0 and width <= 110 and force >= 3 and force <= 40:
                progrg2 = ("def rg2grpCntrl():\n")
                progrg2 += ("	textmsg(\"inside RG2 function called\")\n")
                progrg2 += ("	target_width=" + str(width) + "\n")
                progrg2 += ("	target_force=" + str(force) + "\n")
                progrg2 += ("	timeoutLength=400\n")
                progrg2 += ("	timeoutLengthSecondary=20\n")
                progrg2 += ("	payload=1.0\n")
                progrg2 += ("	set_payload1=False\n")
                progrg2 += ("	depth_compensation=False\n")
                progrg2 += ("	slave=False\n")
                progrg2 += ("	timeout = 0\n")
                progrg2 += ("	while get_digital_in(9) == False:\n")
                progrg2 += ("		textmsg(\"inside while\")\n")
                progrg2 += ("		if timeout > timeoutLength:\n")
                progrg2 += ("			break\n")
                progrg2 += ("		end\n")
                progrg2 += ("		timeout = timeout+1\n")
                progrg2 += ("		sync()\n")
                progrg2 += ("	end\n")
                progrg2 += ("	textmsg(\"outside while\")\n")
                progrg2 += ("	def bit(input):\n")
                progrg2 += ("		msb=65536\n")
                progrg2 += ("		local i=0\n")
                progrg2 += ("		local output=0\n")
                progrg2 += ("		while i<17:\n")
                progrg2 += ("			set_digital_out(8,True)\n")
                progrg2 += ("			if input>=msb:\n")
                progrg2 += ("				input=input-msb\n")
                progrg2 += ("				set_digital_out(9,False)\n")
                progrg2 += ("			else:\n")
                progrg2 += ("				set_digital_out(9,True)\n")
                progrg2 += ("			end\n")
                progrg2 += ("			if get_digital_in(8):\n")
                progrg2 += ("				out=1\n")
                progrg2 += ("			end\n")
                progrg2 += ("			sync()\n")
                progrg2 += ("			set_digital_out(8,False)\n")
                progrg2 += ("			sync()\n")
                progrg2 += ("			input=input*2\n")
                progrg2 += ("			output=output*2\n")
                progrg2 += ("			i=i+1\n")
                progrg2 += ("		end\n")
                progrg2 += ("		return output\n")
                progrg2 += ("	end\n")
                progrg2 += ("	textmsg(\"outside bit definition\")\n")
                progrg2 += ("	target_width=target_width+0.0\n")
                progrg2 += ("	if target_force>40:\n")
                progrg2 += ("		target_force=40\n")
                progrg2 += ("	end\n")
                progrg2 += ("	if target_force<4:\n")
                progrg2 += ("		target_force=4\n")
                progrg2 += ("	end\n")
                progrg2 += ("	if target_width>110:\n")
                progrg2 += ("		target_width=110\n")
                progrg2 += ("	end\n")
                progrg2 += ("	if target_width<0:\n")
                progrg2 += ("		target_width=0\n")
                progrg2 += ("	end\n")
                progrg2 += ("	rg_data=floor(target_width)*4\n")
                progrg2 += ("	rg_data=rg_data+floor(target_force/2)*4*111\n")
                progrg2 += ("	if slave:\n")
                progrg2 += ("		rg_data=rg_data+16384\n")
                progrg2 += ("	end\n")
                progrg2 += ("	textmsg(\"about to call bit\")\n")
                progrg2 += ("	bit(rg_data)\n")
                progrg2 += ("	textmsg(\"called bit\")\n")
                progrg2 += ("	if depth_compensation:\n")
                progrg2 += ("		finger_length = 55.0/1000\n")
                progrg2 += ("		finger_heigth_disp = 5.0/1000\n")
                progrg2 += ("		center_displacement = 7.5/1000\n")
                progrg2 += ("		start_pose = get_forward_kin()\n")
                progrg2 += ("		set_analog_inputrange(2, 1)\n")
                progrg2 += ("		zscale = (get_analog_in(2)-0.026)/2.976\n")
                progrg2 += ("		zangle = zscale*1.57079633-0.087266462\n")
                progrg2 += ("		zwidth = 5+110*sin(zangle)\n")
                progrg2 += ("		start_depth = cos(zangle)*finger_length\n")
                progrg2 += ("		sync()\n")
                progrg2 += ("		sync()\n")
                progrg2 += ("		timeout = 0\n")
                progrg2 += ("		while get_digital_in(9) == True:\n")
                progrg2 += ("			timeout=timeout+1\n")
                progrg2 += ("			sync()\n")
                progrg2 += ("			if timeout > timeoutLengthSecondary:\n")
                progrg2 += ("				break\n")
                progrg2 += ("			end\n")
                progrg2 += ("		end\n")
                progrg2 += ("		timeout = 0\n")
                progrg2 += ("		while get_digital_in(9) == False:\n")
                progrg2 += ("			zscale = (get_analog_in(2)-0.026)/2.976\n")
                progrg2 += ("			zangle = zscale*1.57079633-0.087266462\n")
                progrg2 += ("			zwidth = 5+110*sin(zangle)\n")
                progrg2 += ("			measure_depth = cos(zangle)*finger_length\n")
                progrg2 += ("			compensation_depth = (measure_depth - start_depth)\n")
                progrg2 += ("			target_pose = pose_trans(start_pose,p[0,0,-compensation_depth,0,0,0])\n")
                progrg2 += ("			if timeout > timeoutLength:\n")
                progrg2 += ("				break\n")
                progrg2 += ("			end\n")
                progrg2 += ("			timeout=timeout+1\n")
                progrg2 += ("			servoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n")
                progrg2 += ("		end\n")
                progrg2 += ("		nspeed = norm(get_actual_tcp_speed())\n")
                progrg2 += ("		while nspeed > 0.001:\n")
                progrg2 += ("			servoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n")
                progrg2 += ("			nspeed = norm(get_actual_tcp_speed())\n")
                progrg2 += ("		end\n")
                progrg2 += ("	end\n")
                progrg2 += ("	if depth_compensation==False:\n")
                progrg2 += ("		timeout = 0\n")
                progrg2 += ("		while get_digital_in(9) == True:\n")
                progrg2 += ("			timeout = timeout+1\n")
                progrg2 += ("			sync()\n")
                progrg2 += ("			if timeout > timeoutLengthSecondary:\n")
                progrg2 += ("				break\n")
                progrg2 += ("			end\n")
                progrg2 += ("		end\n")
                progrg2 += ("		timeout = 0\n")
                progrg2 += ("		while get_digital_in(9) == False:\n")
                progrg2 += ("			timeout = timeout+1\n")
                progrg2 += ("			sync()\n")
                progrg2 += ("			if timeout > timeoutLength:\n")
                progrg2 += ("				break\n")
                progrg2 += ("			end\n")
                progrg2 += ("		end\n")
                progrg2 += ("	end\n")
                progrg2 += ("	if set_payload1:\n")
                progrg2 += ("		if slave:\n")
                progrg2 += ("			if get_analog_in(3) < 2:\n")
                progrg2 += ("				zslam=0\n")
                progrg2 += ("			else:\n")
                progrg2 += ("				zslam=payload\n")
                progrg2 += ("			end\n")
                progrg2 += ("		else:\n")
                progrg2 += ("			if get_digital_in(8) == False:\n")
                progrg2 += ("				zmasm=0\n")
                progrg2 += ("			else:\n")
                progrg2 += ("				zmasm=payload\n")
                progrg2 += ("			end\n")
                progrg2 += ("		end\n")
                progrg2 += ("		zsysm=0.0\n")
                progrg2 += ("		zload=zmasm+zslam+zsysm\n")
                progrg2 += ("		set_payload(zload)\n")
                progrg2 += ("	end\n")
                progrg2 += ("end\n")
                self.Robot.send_program(progrg2)
            else:
                self.logger.debug("Width is required to be between 0 and 110 and force is required to be 3 and 40")
                raise RobotException(
                    "Please ensure the gripper width is between 0 and 110 and the force is between 3 and 40")
        except:
            raise RobotException("An unexpected error occured")
        return
