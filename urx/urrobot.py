"""
Python library to control an UR robot through its TCP/IP interface
Documentation from universal robots:
http://support.universal-robots.com/URRobot/RemoteAccess
"""

import logging

from urx import urrtmon
from urx import ursecmon

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2015, Sintef Raufoss Manufacturing"
__license__ = "LGPLv3"


class RobotException(Exception):
    pass


class URRobot(object):

    """
    Python interface to socket interface of UR robot.
    programs are send to port 3002
    data is read from secondary interface(10Hz?) and real-time interface(125Hz) (called Matlab interface in documentation)
    Since parsing the RT interface uses som CPU, and does not support all robots versions, it is disabled by default
    The RT interfaces is only used for the get_force related methods
    Rmq: A program sent to the robot i executed immendiatly and any running program is stopped
    """

    def __init__(self, host, use_rt=False):
        self.logger = logging.getLogger("urx")
        self.host = host
        self.csys = None

        self.logger.debug("Opening secondary monitor socket")
        self.secmon = ursecmon.SecondaryMonitor(self.host)  # data from robot at 10Hz

        self.rtmon = None
        if use_rt:
            self.rtmon = self.get_realtime_monitor()
        # precision of joint movem used to wait for move completion
        # the value must be conservative! otherwise we may wait forever
        self.joinEpsilon = 0.01
        # It seems URScript is  limited in the character length of floats it accepts
        self.max_float_length = 6  # FIXME: check max length!!!

        self.secmon.wait()  # make sure we get data from robot before letting clients access our methods

    def __repr__(self):
        return "Robot Object (IP=%s, state=%s)" % (self.host, self.secmon.get_all_data()["RobotModeData"])

    def __str__(self):
        return self.__repr__()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def is_running(self):
        """
        Return True if robot is running (not
        necessary running a program, it might be idle)
        """
        return self.secmon.running

    def is_program_running(self):
        """
        check if program is running.
        Warning!!!!!:  After sending a program it might take several 10th of
        a second before the robot enters the running state
        """
        return self.secmon.is_program_running()

    def send_program(self, prog):
        """
        send a complete program using urscript to the robot
        the program is executed immediatly and any runnning
        program is interrupted
        """
        self.logger.info("Sending program: " + prog)
        self.secmon.send_program(prog)

    def get_tcp_force(self, wait=True):
        """
        return measured force in TCP
        if wait==True, waits for next packet before returning
        """
        return self.rtmon.getTCFForce(wait)

    def get_force(self, wait=True):
        """
        length of force vector returned by get_tcp_force
        if wait==True, waits for next packet before returning
        """
        tcpf = self.get_tcp_force(wait)
        force = 0
        for i in tcpf:
            force += i**2
        return force**0.5

    def set_tcp(self, tcp):
        """
        set robot flange to tool tip transformation
        """
        prog = "set_tcp(p[{}, {}, {}, {}, {}, {}])".format(*tcp)
        self.send_program(prog)

    def set_payload(self, weight, cog=None):
        """
        set payload in Kg
        cog is a vector x,y,z
        if cog is not specified, then tool center point is used
        """
        if cog:
            cog = list(cog)
            cog.insert(0, weight)
            prog = "set_payload({}, ({},{},{}))".format(*cog)
        else:
            prog = "set_payload(%s)" % weight
        self.send_program(prog)

    def set_gravity(self, vector):
        """
        set direction of gravity
        """
        prog = "set_gravity(%s)" % list(vector)
        self.send_program(prog)

    def send_message(self, msg):
        """
        send message to the GUI log tab on the robot controller
        """
        prog = "textmsg(%s)" % msg
        self.send_program(prog)

    def set_digital_out(self, output, val):
        """
        set digital output. val is a bool
        """
        if val in (True, 1):
            val = "True"
        else:
            val = "False"
        self.send_program('digital_out[%s]=%s' % (output, val))

    def get_analog_inputs(self):
        """
        get analog input
        """
        return self.secmon.get_analog_inputs()

    def get_analog_in(self, nb, wait=False):
        """
        get analog input
        """
        return self.secmon.get_analog_in(nb, wait=wait)

    def get_digital_in_bits(self):
        """
        get digital output
        """
        return self.secmon.get_digital_in_bits()

    def get_digital_in(self, nb, wait=False):
        """
        get digital output
        """
        return self.secmon.get_digital_in(nb, wait)

    def get_digital_out(self, val, wait=False):
        """
        get digital output
        """
        return self.secmon.get_digital_out(val, wait=wait)

    def get_digital_out_bits(self, wait=False):
        """
        get digital output as a byte
        """
        return self.secmon.get_digital_out_bits(wait=wait)

    def set_analog_out(self, output, val):
        """
        set analog output, val is a float
        """
        prog = "set_analog_out(%s, %s)" % (output, val)
        self.send_program(prog)

    def set_tool_voltage(self, val):
        """
        set voltage to be delivered to the tool, val is 0, 12 or 24
        """
        prog = "set_tool_voltage(%s)" % (val)
        self.send_program(prog)

    def _wait_for_move(self, target, threshold=None, timeout=5, joints=False):
        """
        wait for a move to complete. Unfortunately there is no good way to know when a move has finished
        so for every received data from robot we compute a dist equivalent and when it is lower than
        'threshold' we return.
        if threshold is not reached within timeout, an exception is raised
        """
        self.logger.debug("Waiting for move completion using threshold %s and target %s", threshold, target)
        start_dist = self._get_dist(target, joints)
        if threshold is None:
            threshold = start_dist * 0.8
            if threshold < 0.001:  # roboten precision is limited
                threshold = 0.001
            self.logger.debug("No threshold set, setting it to %s", threshold)
        count = 0
        while True:
            if not self.is_running():
                raise RobotException("Robot stopped")
            dist = self._get_dist(target, joints)
            self.logger.debug("distance to target is: %s, target dist is %s", dist, threshold)
            if not self.secmon.is_program_running():
                if dist < threshold:
                    self.logger.debug("we are threshold(%s) close to target, move has ended", threshold)
                    return
                count += 1
                if count > timeout * 10:
                    raise RobotException("Goal not reached but no program has been running for {} seconds. dist is {}, threshold is {}, target is {}, current pose is {}".format(timeout, dist, threshold, target, URRobot.getl(self)))
            else:
                count = 0

    def _get_dist(self, target, joints=False):
        if joints:
            return self._get_joints_dist(target)
        else:
            return self._get_lin_dist(target)

    def _get_lin_dist(self, target):
        # FIXME: we have an issue here, it seems sometimes the axis angle received from robot
        pose = URRobot.getl(self, wait=True)
        dist = 0
        for i in range(3):
            dist += (target[i] - pose[i]) ** 2
        for i in range(3, 6):
            dist += ((target[i] - pose[i]) / 5) ** 2  # arbitraty length like
        return dist ** 0.5

    def _get_joints_dist(self, target):
        joints = self.getj(wait=True)
        dist = 0
        for i in range(6):
            dist += (target[i] - joints[i]) ** 2
        return dist ** 0.5

    def getj(self, wait=False):
        """
        get joints position
        """
        jts = self.secmon.get_joint_data(wait)
        return [jts["q_actual0"], jts["q_actual1"], jts["q_actual2"], jts["q_actual3"], jts["q_actual4"], jts["q_actual5"]]

    def speedx(self, command, velocities, acc, min_time):
        vels = [round(i, self.max_float_length) for i in velocities]
        vels.append(acc)
        vels.append(min_time)
        prog = "{}([{},{},{},{},{},{}], a={}, t_min={})".format(command, *vels)
        self.send_program(prog)

    def movej(self, joints, acc=0.1, vel=0.05, wait=True, relative=False, threshold=None):
        """
        move in joint space
        """
        if relative:
            l = self.getj()
            joints = [v + l[i] for i, v in enumerate(joints)]
        prog = self._format_move("movej", joints, acc, vel)
        self.send_program(prog)
        if wait:
            self._wait_for_move(joints[:6], threshold=threshold, joints=True)
            return self.getj()

    def movel(self, tpose, acc=0.01, vel=0.01, wait=True, relative=False, threshold=None):
        """
        Send a movel command to the robot. See URScript documentation.
        """
        return self.movex("movel", tpose, acc=acc, vel=vel, wait=wait, relative=relative, threshold=threshold)

    def movep(self, tpose, acc=0.01, vel=0.01, wait=True, relative=False, threshold=None):
        """
        Send a movep command to the robot. See URScript documentation.
        """
        return self.movex("movep", tpose, acc=acc, vel=vel, wait=wait, relative=relative, threshold=threshold)

    def servoc(self, tpose, acc=0.01, vel=0.01, wait=True, relative=False, threshold=None):
        """
        Send a servoc command to the robot. See URScript documentation.
        """
        return self.movex("servoc", tpose, acc=acc, vel=vel, wait=wait, relative=relative, threshold=threshold)

    def _format_move(self, command, tpose, acc, vel, radius=0, prefix=""):
        tpose = [round(i, self.max_float_length) for i in tpose]
        tpose.append(acc)
        tpose.append(vel)
        tpose.append(radius)
        return "{}({}[{},{},{},{},{},{}], a={}, v={}, r={})".format(command, prefix, *tpose)

    def movex(self, command, tpose, acc=0.01, vel=0.01, wait=True, relative=False, threshold=None):
        """
        Send a move command to the robot. since UR robotene have several methods this one
        sends whatever is defined in 'command' string
        """
        if relative:
            l = self.getl()
            tpose = [v + l[i] for i, v in enumerate(tpose)]
        prog = self._format_move(command, tpose, acc, vel, prefix="p")
        self.send_program(prog)
        if wait:
            self._wait_for_move(tpose[:6], threshold=threshold)
            return self.getl()

    def getl(self, wait=False, _log=True):
        """
        get TCP position
        """
        pose = self.secmon.get_cartesian_info(wait)
        if pose:
            pose = [pose["X"], pose["Y"], pose["Z"], pose["Rx"], pose["Ry"], pose["Rz"]]
        if _log:
            self.logger.debug("Received pose from robot: %s", pose)
        return pose

    def movec(self, pose_via, pose_to, acc=0.01, vel=0.01, wait=True, threshold=None):
        """
        Move Circular: Move to position (circular in tool-space)
        see UR documentation
        """
        pose_via = [round(i, self.max_float_length) for i in pose_via]
        pose_to = [round(i, self.max_float_length) for i in pose_to]
        prog = "movec(p%s, p%s, a=%s, v=%s, r=%s)" % (pose_via, pose_to, acc, vel, "0")
        self.send_program(prog)
        if wait:
            self._wait_for_move(pose_to, threshold=threshold)
            return self.getl()

    def movels(self, pose_list, acc=0.01, vel=0.01, radius=0.01, wait=True, threshold=None):
        """
        Concatenate several movel commands and applies a blending radius
        pose_list is a list of pose.
        This method is usefull since any new command from python
        to robot make the robot stop
        """
        return self.movexs("movel", pose_list, acc, vel, radius, wait, threshold=threshold)

    def movexs(self, command, pose_list, acc=0.01, vel=0.01, radius=0.01, wait=True, threshold=None):
        """
        Concatenate several movex commands and applies a blending radius
        pose_list is a list of pose.
        This method is usefull since any new command from python
        to robot make the robot stop
        """
        header = "def myProg():\n"
        end = "end\n"
        prog = header
        for idx, pose in enumerate(pose_list):
            if idx == (len(pose_list) - 1):
                radius = 0
            prog += self._format_move(command, pose, acc, vel, radius, prefix="p") + "\n"
        prog += end
        self.send_program(prog)
        if wait:
            self._wait_for_move(target=pose_list[-1], threshold=threshold)
            return self.getl()

    def stopl(self, acc=0.5):
        self.send_program("stopl(%s)" % acc)

    def stopj(self, acc=1.5):
        self.send_program("stopj(%s)" % acc)

    def stop(self):
        self.stopj()

    def close(self):
        """
        close connection to robot and stop internal thread
        """
        self.logger.info("Closing sockets to robot")
        self.secmon.close()
        if self.rtmon:
            self.rtmon.stop()

    def set_freedrive(self, val, timeout=60):
        """
        set robot in freedrive/backdrive mode where an operator can jog
        the robot to wished pose.

        Freedrive will timeout at 60 seconds.
        """
        if val:
            self.send_program("def myProg():\n\tfreedrive_mode()\n\tsleep({})\nend".format(timeout))
        else:
            # This is a non-existant program, but running it will stop freedrive
            self.send_program("def myProg():\n\tend_freedrive_mode()\nend")

    def set_simulation(self, val):
        if val:
            self.send_program("set sim")
        else:
            self.send_program("set real")

    def get_realtime_monitor(self):
        """
        return a pointer to the realtime monitor object
        usefull to track robot position for example
        """
        if not self.rtmon:
            self.logger.info("Opening real-time monitor socket")
            self.rtmon = urrtmon.URRTMonitor(self.host)  # som information is only available on rt interface
            self.rtmon.start()
        self.rtmon.set_csys(self.csys)
        return self.rtmon

    def translate(self, vect, acc=0.01, vel=0.01, wait=True, command="movel"):
        """
        move tool in base coordinate, keeping orientation
        """
        p = self.getl()
        p[0] += vect[0]
        p[1] += vect[1]
        p[2] += vect[2]
        return self.movex(command, p, vel=vel, acc=acc, wait=wait)

    def up(self, z=0.05, acc=0.01, vel=0.01):
        """
        Move up in csys z
        """
        p = self.getl()
        p[2] += z
        self.movel(p, acc=acc, vel=vel)

    def down(self, z=0.05, acc=0.01, vel=0.01):
        """
        Move down in csys z
        """
        self.up(-z, acc, vel)

    def move_RG2gripper(self,width):
	"""
	
	Created by David Hinwood, University of Canberra
	This function opens and closes the RG2 gripper produced by on robot
	This work was inspired by Mark Silliman who produced a class to control
	a gripper made by robotiq and Sharath Jotawar who created a ROS package
	used in moveit to control the RG2 gripper 
	"""
        progrg2 = ""
        try:
            if width >= 0 and width <= 110:
                progrg2 += ("def rg2grpCntrl():\n")
                progrg2 += ("	textmsg(\"inside RG2 function called\")\n")
                progrg2 += ("	target_width=" + str(width)) + "\n"
                progrg2 += ("	target_force=40\n")
                progrg2 += ("	payload=1.0\n")
                progrg2 += ("	set_payload1=False\n")
                progrg2 += ("	depth_compensation=False\n")
                progrg2 += ("	slave=False\n")
                progrg2 += ("	timeout = 0\n")
                progrg2 += ("	while get_digital_in(9) == False:\n")
                progrg2 += ("		textmsg(\"inside while\")\n")
                progrg2 += ("		if timeout > 400:\n")
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
                progrg2 += ("			if timeout > 20:\n")
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
                progrg2 += ("			if timeout > 400:\n")
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
                progrg2 += ("			if timeout > 20:\n")
                progrg2 += ("				break\n")
                progrg2 += ("			end\n")
                progrg2 += ("		end\n")
                progrg2 += ("		timeout = 0\n")
                progrg2 += ("		while get_digital_in(9) == False:\n")
                progrg2 += ("			timeout = timeout+1\n")
                progrg2 += ("			sync()\n")
                progrg2 += ("			if timeout > 400:\n")
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
                self.send_program(progrg2)
            else:
                self.logger.debug("Width is required to be between 0 and 110")
                raise RobotException("Please ensure the gripper width is between 0 and 110")
        except:
            raise RobotException("An unexpected error occured")

