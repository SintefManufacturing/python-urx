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
__license__ = "GPLv3"


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
        prog = "set_analog_output(%s, %s)" % (output, val)
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
            if threshold < 0.001: # roboten precision is limited
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
        #FIXME: we have an issue here, it seems sometimes the axis angle received from robot
        pose = URRobot.getl(self, wait=True)
        dist = 0
        for i in range(3):
            dist += (target[i] - pose[i]) ** 2
        for i in range(3, 6):
            dist += ((target[i] - pose[i]) / 5) ** 2 # arbitraty length like
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

    def speedl(self, velocities, acc, min_time):
        """
        move at given velocities until minimum min_time seconds
        """
        return self.speedx("speedl", velocities, acc, min_time)

    def speedj(self, velocities, acc, min_time):
        """
        move at given joint velocities until minimum min_time seconds
        """
        return self.speedx("speedj", velocities, acc, min_time)

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

    def set_freedrive(self, val):
        """
        set robot in freedrive/brackdrive mode where an operator can jogg
        the robot to wished pose
        """
        if val:
            self.send_program("set robotmode freedrive")
        else:
            self.send_program("set robotmode run")

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



