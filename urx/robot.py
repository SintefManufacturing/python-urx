"""
Python library to control an UR robot through its TCP/IP interface
DOC LINK
http://support.universal-robots.com/URRobot/RemoteAccess
"""

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2015, Sintef Raufoss Manufacturing"
__license__ = "GPLv3"

import logging


MATH3D = True
try:
    import math3d as m3d
    import numpy as np
except ImportError:
    MATH3D = False
    print("python-math3d library could not be found on this computer, disabling use of matrices and path blending")

from urx import urrtmon
from urx import ursecmon


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

    #def wait_for_move(self, radius=0, target=None):
    def wait_for_move(self, radius, target):
        """
        wait until a move is completed
        radius and target args are ignored
        """
        try:
            self._wait_for_move(radius, target)
        except Exception as ex:
            self.logger.exception("Exception:")
            self.stopj()
            raise ex

    def _wait_for_move(self, radius, target):
        self.logger.debug("Waiting for move completion with radius %s and target %s", radius, target)
        # it is necessary to wait since robot may takes a while to get into running state,
        for _ in range(3):
            self.secmon.wait()
        while True:
            if not self.is_running():
                raise RobotException("Robot stopped")
            jts = self.secmon.get_joint_data(wait=True)
            finished = True
            for i in range(0, 6):
                # Rmq: q_target is an interpolated target we have no control over
                if abs(jts["q_actual%s" % i] - jts["q_target%s" % i]) > self.joinEpsilon:
                    self.logger.debug("Waiting for end move, q_actual is %s, q_target is %s, diff is %s, epsilon is %s", jts["q_actual%s" % i], jts["q_target%s" % i], jts["q_actual%s" % i] - jts["q_target%s" % i], self.joinEpsilon)
                    finished = False
                    break
            if finished and not self.secmon.is_program_running():
                self.logger.debug("move has ended")
                return

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

    def movej(self, joints, acc=0.1, vel=0.05, radius=0, wait=True, relative=False):
        """
        move in joint space
        """
        if relative:
            l = self.getj()
            joints = [v + l[i] for i, v in enumerate(joints)]
        prog = self._format_move("movej", joints, acc, vel, radius)
        self.send_program(prog)
        if wait:
            self.wait_for_move(radius, joints[:6])
            return self.getj()

    def movel(self, tpose, acc=0.01, vel=0.01, radius=0, wait=True, relative=False):
        """
        Send a movel command to the robot. See URScript documentation.
        """
        return self.movex("movel", tpose, acc=acc, vel=vel, radius=radius, wait=wait, relative=relative)

    def movep(self, tpose, acc=0.01, vel=0.01, radius=0, wait=True, relative=False):
        """
        Send a movep command to the robot. See URScript documentation.
        """
        return self.movex("movep", tpose, acc=acc, vel=vel, radius=radius, wait=wait, relative=relative)

    def servoc(self, tpose, acc=0.01, vel=0.01, radius=0, wait=True, relative=False):
        """
        Send a servoc command to the robot. See URScript documentation.
        """
        return self.movex("servoc", tpose, acc=acc, vel=vel, radius=radius, wait=wait, relative=relative)

    def _format_move(self, command, tpose, acc, vel, radius, prefix=""):
        tpose = [round(i, self.max_float_length) for i in tpose]
        tpose.append(acc)
        tpose.append(vel)
        tpose.append(radius)
        return "{}({}[{},{},{},{},{},{}], a={}, v={}, r={})".format(command, prefix, *tpose)

    def movex(self, command, tpose, acc=0.01, vel=0.01, radius=0, wait=True, relative=False):
        """
        Send a move command to the robot. since UR robotene have several methods this one
        sends whatever is defined in 'command' string
        """
        if relative:
            l = self.getl()
            tpose = [v + l[i] for i, v in enumerate(tpose)]
        prog = self._format_move(command, tpose, acc, vel, radius, prefix="p")
        self.send_program(prog)
        if wait:
            self.wait_for_move(radius, tpose[:6])
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

    def movec(self, pose_via, pose_to, acc=0.01, vel=0.01, radius=0, wait=True):
        """
        Move Circular: Move to position (circular in tool-space)
        see UR documentation
        """
        pose_via = [round(i, self.max_float_length) for i in pose_via]
        pose_to = [round(i, self.max_float_length) for i in pose_to]
        prog = "movec(p%s, p%s, a=%s, v=%s, r=%s)" % (pose_via, pose_to, acc, vel, radius)
        self.send_program(prog)
        if wait:
            self.wait_for_move(radius, pose_to)
            return self.getl()

    def movels(self, pose_list, acc=0.01, vel=0.01, radius=0.01, wait=True):
        """
        Concatenate several movel commands and applies a blending radius
        pose_list is a list of pose.
        This method is usefull since any new command from python
        to robot make the robot stop
        """
        return self.movexs("movel", pose_list, acc, vel, radius, wait)

    def movexs(self, command, pose_list, acc=0.01, vel=0.01, radius=0.01, wait=True):
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
            self.wait_for_move(radius=0, target=pose_list[-1])
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



class Robot(URRobot):

    """
    Generic Python interface to an industrial robot.
    Compared to the URRobot class, this class adds the possibilty to work directly with matrices
    and includes support for setting a reference coordinate system
    """

    def __init__(self, host, use_rt=False):
        URRobot.__init__(self, host, use_rt)
        self.csys = m3d.Transform()

    def set_tcp(self, tcp):
        """
        set robot flange to tool tip transformation
        """
        if isinstance(tcp, m3d.Transform):
            tcp = tcp.pose_vector
        URRobot.set_tcp(self, tcp)

    def set_csys(self, transform):
        """
        Set reference coordinate system to use
        """
        self.csys = transform

    def set_orientation(self, orient, acc=0.01, vel=0.01, radius=0, wait=True):
        """
        set tool orientation using a orientation matric from math3d
        or a orientation vector
        """
        if not isinstance(orient, m3d.Orientation):
            orient = m3d.Orientation(orient)
        trans = self.get_pose()
        trans.orient = orient
        self.set_pose(trans, acc, vel, radius, wait=wait)

    def translate_tool(self, vect, acc=0.01, vel=0.01, radius=0, wait=True):
        """
        move tool in tool coordinate, keeping orientation
        """
        t = m3d.Transform()
        if not isinstance(vect, m3d.Vector):
            vect = m3d.Vector(vect)
        t.pos += vect
        return self.add_pose_tool(t, acc, vel, radius, wait=wait)

    def back(self, z=0.05, acc=0.01, vel=0.01):
        """
        move in z tool
        """
        self.translate_tool((0, 0, -z), acc=acc, vel=vel)

    def set_pos(self, vect, acc=0.01, vel=0.01, radius=0, wait=True):
        """
        set tool to given pos, keeping constant orientation
        """
        if not isinstance(vect, m3d.Vector):
            vect = m3d.Vector(vect)
        trans = m3d.Transform(self.get_orientation(), m3d.Vector(vect))
        return self.set_pose(trans, acc, vel, radius, wait=wait)

    def movec(self, pose_via, pose_to, acc=0.01, vel=0.01, radius=0, wait=True):
        """
        Move Circular: Move to position (circular in tool-space)
        see UR documentation
        """
        pose_via = self.csys * m3d.Transform(pose_via)
        pose_to = self.csys * m3d.Transform(pose_to)
        pose = URRobot.movec(self, pose_via.pose_vector, pose_to.pose_vector, acc=acc, vel=vel, wait=wait, radius=radius)
        if pose is not None:
            return self.csys.inverse * m3d.Transform(pose)

    def set_pose(self, trans, acc=0.01, vel=0.01, radius=0, wait=True, command="movel"):
        """
        move tcp to point and orientation defined by a transformation
        UR robots have several move commands, by default movel is used but it can be changed
        using the command argument
        """
        self.logger.debug("Setting pose to %s", trans.pose_vector)
        t = self.csys * trans
        pose = URRobot.movex(self, command, t.pose_vector, acc=acc, vel=vel, wait=wait, radius=radius)
        if pose is not None:
            return self.csys.inverse * m3d.Transform(pose)

    def add_pose_base(self, trans, acc=0.01, vel=0.01, radius=0, wait=True, command="movel"):
        """
        Add transform expressed in base coordinate
        """
        pose = self.get_pose()
        return self.set_pose(trans * pose, acc, vel, radius, wait=wait, command=command)

    def add_pose_tool(self, trans, acc=0.01, vel=0.01, radius=0, wait=True, command="movel"):
        """
        Add transform expressed in tool coordinate
        """
        pose = self.get_pose()
        return self.set_pose(pose * trans, acc, vel, radius, wait=wait, command=command)

    def get_pose(self, wait=False, _log=True):
        """
        get current transform from base to to tcp
        """
        pose = URRobot.getl(self, wait, _log)
        trans = self.csys.inverse * m3d.Transform(pose)
        if _log:
            self.logger.debug("Returning pose to user: %s", trans.pose_vector)
        return trans

    def get_orientation(self, wait=False):
        """
        get tool orientation in base coordinate system
        """
        trans = self.get_pose(wait)
        return trans.orient

    def get_pos(self, wait=False):
        """
        get tool tip pos(x, y, z) in base coordinate system
        """
        trans = self.get_pose(wait)
        return trans.pos

    def speedx(self, command, velocities, acc, min_time):
        """
        send command to robot formated like speedl or speedj
        move at given velocities until minimum min_time seconds
        """
        v = self.csys.orient * m3d.Vector(velocities[:3])
        w = self.csys.orient * m3d.Vector(velocities[3:])
        URRobot.speedx(self, command, np.concatenate((v.array, w.array)), acc, min_time)

    def speedl_tool(self, velocities, acc, min_time):
        """
        move at given velocities in tool csys until minimum min_time seconds
        """
        pose = self.get_pose()
        v = self.csys.orient * pose.orient * m3d.Vector(velocities[:3])
        w = self.csys.orient * pose.orient * m3d.Vector(velocities[3:])
        URRobot.speedl(self, np.concatenate((v.array, w.array)), acc, min_time)

    def movex(self, command, pose, acc=0.01, vel=0.01, wait=True, relative=False, radius=0):
        """
        Send a move command to the robot. since UR robotene have several methods this one
        sends whatever is defined in 'command' string
        """
        t = m3d.Transform(pose)
        if relative:
            return self.add_pose_base(t, acc, vel, wait=wait, command=command)
        else:
            return self.set_pose(t, acc, vel, radius, wait=wait, command=command)

    def movexs(self, command, pose_list, acc=0.01, vel=0.01, radius=0.01, wait=True):
        """
        Concatenate several movex commands and applies a blending radius
        pose_list is a list of pose.
        This method is usefull since any new command from python
        to robot make the robot stop
        """
        new_poses = []
        for pose in pose_list:
            t = self.csys * m3d.Transform(pose)
            pose = t.pose_vector
            new_poses.append(pose)
        return URRobot.movexs(self, command, new_poses, acc, vel, radius, wait=wait)

    def movel_tool(self, pose, acc=0.01, vel=0.01, wait=True):
        """
        move linear to given pose in tool coordinate
        """
        return self.movex_tool("movel", pose, acc=acc, vel=vel, wait=wait)

    def movex_tool(self, command, pose, acc=0.01, vel=0.01, wait=True):
        t = m3d.Transform(pose)
        self.add_pose_tool(t, acc, vel, wait=wait, command=command)

    def getl(self, wait=False, _log=True):
        """
        return current transformation from tcp to current csys
        """
        t = self.get_pose(wait, _log)
        return t.pose_vector.tolist()

    def set_gravity(self, vector):
        if isinstance(vector, m3d.Vector):
            vector = vector.list
        return URRobot.set_gravity(self, vector)

    def _wait_for_move(self, radius, target):
        self.logger.debug("Waiting for move completion with math3d using raidus %s and target %s", radius, target)
        # it is necessary to wait since robot may takes a while to get into running state,
        for _ in range(3):
            self.secmon.wait()
        target = m3d.Transform(target)
        while True:
            if not self.is_running():
                raise RobotException("Robot stopped")
            pose = self.get_pose(wait=True, _log=False)
            dist = pose.dist(target)
            self.logger.debug("distance to target is: %s, target dist is %s", dist, radius)
            if (dist < radius) or not self.secmon.is_program_running():
            #if (dist < radius):
                self.logger.debug("move has ended")
                return


if not MATH3D:
    Robot = URRobot
