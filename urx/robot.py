"""
Python library to control an UR robot through its TCP/IP interface
DOC LINK
http://support.universal-robots.com/URRobot/RemoteAccess
"""
from __future__ import absolute_import # necessary for import tricks to work with python2

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2013, Sintef Raufoss Manufacturing"
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
    def __init__(self, host, useRTInterface=False, logLevel=logging.WARN, parserLogLevel=logging.WARN):
        self.logger = logging.getLogger(self.__class__.__name__)
        if len(logging.root.handlers) == 0: #dirty hack
            logging.basicConfig()
        self.logger.setLevel(logLevel)
        self.host = host
        self.csys = None 
        
        self.logger.info("Opening secondary monitor socket")
        self.secmon = ursecmon.SecondaryMonitor(self.host, logLevel=logLevel, parserLogLevel=parserLogLevel) #data from robot at 10Hz
       
        self.rtmon = None
        if useRTInterface:
            self.rtmon = self.get_realtime_monitor()
        #the next 3 values must be conservative! otherwise we may wait forever
        self.joinEpsilon = 0.01 # precision of joint movem used to wait for move completion
        # It seems URScript is  limited in the character length of floats it accepts
        self.max_float_length = 6 # FIXME: check max length!!!

        self.secmon.wait() # make sure we get data from robot before letting clients access our methods
        

    def __repr__(self):
        return "Robot Object (IP=%s, state=%s)" % (self.host, self.secmon.get_all_data()["RobotModeData"])

    def __str__(self):
        return self.__repr__()

    def is_running(self): # legacy
        return self.secmon.running

    def is_program_running(self):
        return self.secmon.is_program_running()

    def send_program(self, prog):
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
        tcpf = self.get_tcp_force( wait)
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
        return self.secmon.get_digital_out(val , wait=wait)

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

    def wait_for_move(self, radius=0, target=None):
        """
        wait until a move is completed
        if radius is not 0, returns when tcp is radius closed to target(using mathd3d dist method)
        """
        self.logger.debug("Waiting for move completion")
        # it is necessary to wait since robot may takes a while to get into running state, 
        # for a physical move 0.5s is very short
        for _ in range(3):
            self.secmon.wait()
        if not radius or not MATH3D:
            return self._wait_for_move()
        else:
            return self._wait_for_move_radius(radius, target)

    def _wait_for_move(self):
        while True:
            if not self.is_running():
                raise RobotException("Robot stopped")
            jts = self.secmon.get_joint_data(wait=True)
            finished = True
            for i in range(0, 6):
                #Rmq: q_target is an interpolated target we have no control over
                if abs(jts["q_actual%s"%i] - jts["q_target%s"%i]) > self.joinEpsilon:
                    self.logger.debug("Waiting for end move, q_actual is {}, q_target is {}, diff is {}, epsilon is {}".format( jts["q_actual%s"%i], jts["q_target%s"%i]  , jts["q_actual%s"%i] - jts["q_target%s"%i], self.joinEpsilon))
                    finished = False
                    break
            if finished and not self.secmon.is_program_running():
                self.logger.debug("move has ended")
                return

    def _wait_for_move_radius(self, radius, target):
        print("Wait move with radius")
        target = m3d.Transform(target)
        while True:
            if not self.is_running():
                raise RobotException("Robot stopped")
            pose = self.get_pose(wait=True)
            dist = pose.dist(target)
            print("dist is ", dist, radius)
            if (dist < radius) or not self.secmon.is_program_running():
                self.logger.debug("move has ended")
                return

    def dist(self, poseA, poseB):
        """
        Return the metric distance between two poses as unweighted combined linear
        and angular distance.
        This would be better not to rely on math3d just for that method...
        """
        raise NotImplementedError

    def getj(self, wait=False):
        """
        get joints position
        """
        jts = self.secmon.get_joint_data(wait)
        return [jts["q_actual0"], jts["q_actual1"], jts["q_actual2"], jts["q_actual3"], jts["q_actual4"], jts["q_actual5"]]

    def speedl(self, velocities, acc, min_time):
        """
        move at given velocities until minimum min_time seconds
        """
        vels = [round(i, self.max_float_length) for i in velocities]
        vels.append(acc)
        vels.append(min_time)
        prog = "speedl([{},{},{},{},{},{}], a={}, t_min={})".format(*vels)
        self.send_program(prog)

    def speedj(self, velocities, acc, min_time):
        """
        move at given joint velocities until minimum min_time seconds
        """
        vels = [round(i, self.max_float_length) for i in velocities]
        vels.append(acc)
        vels.append(min_time)
        prog = "speedj([{},{},{},{},{},{}], a={}, t_min={})".format(*vels)
        self.send_program(prog)

    def movej(self, joints, acc=0.1, vel=0.05, radius=0, wait=True, relative=False):
        """
        move in joint space
        """
        if relative:
            l = self.getj()
            joints = [v + l[i] for i, v in enumerate(joints)]
        joints = [round(j, self.max_float_length) for j in joints] 
        joints.append(acc)
        joints.append(vel)
        joints.append(radius)
        prog = "movej([{},{},{},{},{},{}], a={}, v={}, r={})".format(*joints)
        self.send_program(prog)
        if not wait:
            return None
        else:
            self.wait_for_move(radius, joints[:6])
            return self.getj()

    def movel(self, tpose, acc=0.01, vel=0.01, radius=0, wait=True, relative=False):
        """
        linear move 
        """
        if relative:
            l = self.getl()
            tpose = [v + l[i] for i, v in enumerate(tpose)]
        tpose = [round(i, self.max_float_length) for i in tpose]
        #prog = "movel(p%s, a=%s, v=%s)" % (tpose, acc, vel)
        tpose.append(acc)
        tpose.append(vel)
        tpose.append(radius)
        prog = "movel(p[{},{},{},{},{},{}], a={}, v={}, r={})".format(*tpose)
        self.send_program(prog)
        if not wait:
            return None
        else:
            self.wait_for_move(radius, tpose[:6])
            return self.getl()

    def movep(self, tpose, acc=0.01, vel=0.01, radius=0, wait=True, relative=False):
        """
        Send a movep command to the robot. See URScript documentation.
        From URX the main advantage of movep is that it allows for path blending if
        math3d is installed (BROKEN!)
        """
        if relative:
            l = self.getl()
            tpose = [v + l[i] for i, v in enumerate(tpose)]
        tpose = [round(i, self.max_float_length) for i in tpose]
        #prog = "movel(p%s, a=%s, v=%s)" % (tpose, acc, vel)
        tpose.append(acc)
        tpose.append(vel)
        tpose.append(radius)
        prog = "movep(p[{},{},{},{},{},{}], a={}, v={}, r={})".format(*tpose)
        self.send_program(prog)
        if not wait:
            return None
        else:
            self.wait_for_move(radius, tpose[:6])
            return self.getl()

    def getl(self, wait=False):
        """
        get TCP position
        """
        pose = self.secmon.get_cartesian_info(wait)
        if pose:
            pose =  [pose["X"], pose["Y"], pose["Z"], pose["Rx"], pose["Ry"], pose["Rz"]]
        self.logger.debug("Current pose from robot: " + str(pose))
        return pose

    def get_pose(self, wait=False):
        return self.getl(wait)

    def movec(self, pose_via, pose_to, acc=0.01, vel=0.01, radius=0, wait=True):
        """
        Move Circular: Move to position (circular in tool-space)
        see UR documentation
        """
        pose_via = [round(i, self.max_float_length) for i in pose_via]
        pose_to = [round(i, self.max_float_length) for i in pose_to]
        prog = "movec(p%s, p%s, a=%s, v=%s, r=%s)" % (pose_via, pose_to, acc, vel, radius)
        self.send_program(prog)
        if not wait:
            return None
        else:
            self.wait_for_move(radius, pose_to)
            return self.getl()

    def movels(self, pose_list, acc=0.01, vel=0.01 , radius=0.01, wait=True):
        """
        Concatenate several movel commands and applies a blending radius
        pose_list is a list of pose. 
        """
        header = "def myProg():\n"
        end = "end\n"
        template = "movel(p[{},{},{},{},{},{}], a={}, v={}, r={})\n"
        prog = header
        for idx, pose in enumerate(pose_list):
            pose.append(acc)
            pose.append(vel)
            if idx != (len(pose_list) -1 ):
                pose.append(radius)
            else:
                pose.append(0)
            prog += template.format(*pose)
        prog += end
        self.send_program(prog)
        if not wait:
            return None
        else:
            self.wait_for_move()
            return self.getl()

    def stopl(self, acc = 0.5):
        self.send_program("stopl(%s)" % acc)

    def stopj(self, acc = 1.5):
        self.send_program("stopj(%s)" % acc)

    def stop(self):
        self.stopj()

    def cleanup(self):
        self.logger.info("Closing sockets to robot")
        self.secmon.cleanup()
        if self.rtmon:
            self.rtmon.stop()
    shutdown = cleanup #this might be wrong since we could also shutdown the robot hardware from this script

    def set_freedrive(self, val):
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
            self.rtmon = urrtmon.URRTMonitor(self.host)# som information is only available on rt interface
            self.rtmon.start()
        self.rtmon.set_csys(self.csys) 
        return self.rtmon




class Robot(URRobot):
    """
    Generic Python interface to an industrial robot.
    Compared to the URRobot class, this class adds the possibilty to work directly with matrices
    and includes support for calibrating the robot coordinate system
    """
    def __init__(self, host, useRTInterface=False, logLevel = logging.WARN, parserLogLevel=logging.WARN):
        URRobot.__init__(self, host, useRTInterface, logLevel=logLevel, parserLogLevel=parserLogLevel)
        self.default_linear_acceleration = 0.01
        self.default_linear_velocity = 0.01
        self.csys_dict = {}
        self.csys = None
        self.csys_inv = None
        self.set_csys("Robot", m3d.Transform()) #identity

    def set_tcp(self, tcp):
        if type(tcp) == m3d.Transform:
            tcp = tcp.pose_vector
        URRobot.set_tcp(self, tcp)

    def add_csys(self, name, matrix):
        self.csys_dict[name] = matrix

    def get_csys_list(self):
        return self.csys_dict

    def set_csys(self, name, matrix=None):
        """
        Set reference corrdinate system to use
        if matrix != None then a new csys is created
        """
        if matrix:
            self.add_csys(name, matrix)
        self.csys = self.csys_dict[name]
        self.csys_inv = self.csys.inverse()

    def orient(self, orient, acc=None, vel=None, radius=0, wait=True):
        if type(orient) is not m3d.Orientation:
            orient = m3d.Orientation(orient)
        trans = self.get_transform()
        trans.orient = orient
        self.apply_transform(trans, acc, vel, radius, wait=wait)

    set_orientation = orient

    def translate(self, vect, acc=None, vel=None, radius=0, wait=True):
        """
        move tool in base coordinate, keeping orientation
        """
        t = m3d.Transform()
        if not type(vect) is m3d.Vector:
            vect = m3d.Vector(vect)
        t.pos += m3d.Vector(vect)
        return self.add_transform_base(t, acc, vel, radius, wait=wait)

    def translate_tool(self, vect, acc=None, vel=None, radius=0, wait=True):
        """
        move tool in tool coordinate, keeping orientation
        """
        t = m3d.Transform()
        if not type(vect) is m3d.Vector:
            vect = m3d.Vector(vect)
        t.pos += vect
        return self.add_transform_tool(t, acc, vel, radius, wait=wait)

    def set_pos(self, vect, acc=None, vel=None, radius=0, wait=True):
        """
        set tool to given pos, keeping constant orientation
        """
        if not type(vect) is m3d.Vector:
            vect = m3d.Vector(vect)
        trans = m3d.Transform(self.get_orientation(), m3d.Vector(vect))
        return self.apply_transform(trans, acc, vel, radius, wait=wait)

    def apply_transform(self, trans, acc=None, vel=None, radius=0, wait=True, process=False):
        """
        move tcp to point and orientation defined by a transformation
        if process is True, movep is used instead of movel
        if radius is not 0 and wait is True, the method will return when tcp
        is radius close to the target. It is then possible to send another command 
        and the controller will blend the path. Blending only works with process(movep). (BROKEN!)
        """
        if not acc: 
            acc = self.default_linear_acceleration
        if not vel: 
            vel = self.default_linear_velocity
        t = self.csys * trans
        if process:
            pose = URRobot.movep(self, t.pose_vector, acc=acc, vel=vel, wait=wait, radius=radius)
        else:
            pose = URRobot.movel(self, t.pose_vector, acc=acc, vel=vel, wait=wait, radius=radius)
        if pose != None : #movel does not return anything when wait is False
            return self.csys_inv * m3d.Transform(pose)

    set_transform = apply_transform #since we have set_pos, set_orient, we should be consistent and use set for tranform to ..

    def add_transform_base(self, trans, acc=None, vel=None, radius=0, wait=True, process=False):
        """
        Add transform expressed in base coordinate
        """
        pose = self.get_transform()
        return self.apply_transform(trans * pose, acc, vel, radius, wait=wait, process=process)

    def add_transform_tool(self, trans, acc=None, vel=None, radius=0, wait=True, process=False):
        """
        Add transform expressed in tool coordinate
        """
        pose = self.get_transform()
        return self.apply_transform(pose * trans, acc, vel, radius, wait=wait, process=process)

    def get_transform(self, wait=False):
        """
        get current transform from base to to tcp
        """
        pose = URRobot.getl(self, wait)
        trans = self.csys_inv * m3d.Transform(pose) 
        return trans

    def get_pose(self, wait=False):
        """
        get current transform from base to to tcp
        """
        return self.get_transform(wait)

    def get_orientation(self, wait=False):
        trans  = self.get_transform(wait)
        return trans.orient

    def get_pos(self, wait=False):
        trans  = self.get_transform(wait)
        return trans.pos

    def speedl(self, velocities, acc, min_time):
        """
        move at given velocities until minimum min_time seconds
        """
        v = self.csys.orient * m3d.Vector(velocities[:3])
        w = self.csys.orient * m3d.Vector(velocities[3:])
        URRobot.speedl(self, np.concatenate((v.data, w.data)), acc, min_time)

    def speedl_tool(self, velocities, acc, min_time):
        """
        move in tool coordinate at given velocities until minimum min_time seconds
        """
        pose = self.get_transform()
        v = self.csys.orient * pose.orient * m3d.Vector(velocities[:3])
        w = self.csys.orient * pose.orient * m3d.Vector(velocities[3:])
        URRobot.speedl(self, np.concatenate((v.data, w.data)), acc, min_time)

    def movel(self, pose, acc=None, vel=None, wait=True, relative=False, radius=0.01):
        """
        move linear to given pose in current csys
        if pose is a list of poses then movels is called
        """
        t = m3d.Transform(pose)
        if relative:
            return self.add_transform_base(t, acc, vel, wait=wait, process=False)
        else:
            return self.apply_transform(t, acc, vel, radius, wait=wait, process=False)

    def movec(self, pose_via, pose_to, acc=0.01, vel=0.01, radius=0, wait=True):
        """
        Move Circular: Move to position (circular in tool-space)
        see UR documentation
        """
        via = self.csys * m3d.Transform(pose_via)
        to = self.csys * m3d.Transform(pose_to)
        return URRobot.movec(self, via.pose_vector, to.pose_vector, acc=acc, vel=vel, radius=radius, wait=wait)

    def movel_tool(self, pose, acc=None, vel=None, wait=True):
        """
        move linear to given pose in tool coordinate
        """
        t = m3d.Transform(pose)
        self.add_transform_tool(t, acc, vel, wait=wait, process=False)

    def movep(self, pose, acc=None, vel=None, radius=0, wait=True, relative=False):
        pose = m3d.Transform(pose)
        if relative:
            return self.add_transform_base(pose, acc, vel, wait=wait, process=True, radius=radius)
        else:
            return self.apply_transform(pose, acc, vel, wait=wait, process=True, radius=radius)

    def getl(self, wait=False):
        """
        return current transformation from tcp to current csys
        """
        t = self.get_transform(wait)
        return t.pose_vector.tolist()

    def movels(self, pose_list, acc=0.01, vel=0.01 , radius=0.01, wait=True):
        """
        Concatenate several movep commands and applies a blending radius
        pose_list is a list of pose. 
        """
        new_poses = []
        for pose in pose_list:
            t = self.csys * m3d.Transform(pose)
            pose = t.pose_vector
            pose = [round(i, self.max_float_length) for i in pose]
            new_poses.append(pose)
        return URRobot.movels(self, new_poses, acc, vel, radius, wait=wait)

    def set_gravity(self, vector):
        if type(vector) == m3d.Vector:
            vector = vector.list
        return URRobot.set_gravity(self, vector)


if not MATH3D:
    Robot = URRobot

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO) #enable logging
    try:
        #robot = Robot( '192.168.1.6')
        robot = Robot( '192.168.1.5')
        r = robot

        from IPython.frontend.terminal.embed import InteractiveShellEmbed
        ipshell = InteractiveShellEmbed( banner1="\n\n  robot object is available  \n\n")
        ipshell(local_ns=locals())

    finally:
        if "robot" in dir():
            robot.cleanup()




