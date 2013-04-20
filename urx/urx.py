"""
Python library to control an UR robot through its TCP/IP interface
DOC LINK
http://support.universal-robots.com/URRobot/RemoteAccess
"""
from __future__ import absolute_import # necessary for import tricks to work with python2

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2012, Olivier Roulet-Dubonnet"
__credits__ = ["Olivier Roulet-Dubonnet"]
__license__ = "GPLv3"
__version__ = "0.3"
__status__ = "Development"



import time
import logging


MATH3D = True
try:
    import math3d
except ImportError:
    MATH3D = False
    print("pymath3d library could not be found on this computer, disabling use of matrices")

from urx import urrtmon 
from urx import ursecmon
from urx import tracker


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
        
        self.logger.info("Opening secondary monitor socket")
        self.secmon = ursecmon.SecondaryMonitor(self.host, logLevel=logLevel, parserLogLevel=parserLogLevel) #data from robot at 10Hz
        
        if useRTInterface:
            self.logger.info("Opening real-time monitor socket")
            self.rtmon = urrtmon.URRTMonitor(self.host)# som information is only available on rt interface
        else:
            self.rtmon = None
        #the next 3 values must be conservative! otherwise we may wait forever
        self.jointEpsilon = 0.05 # precision of joint movem used to wait for move completion
        self.linearEpsilon = 0.0005 # precision of linear movement, used to wait for move completion
        self.radialEpsilon = 0.05 # precision of radial movement, used to wait for move completion
        #URScript is  limited in the character length of floats it accepts
        self.max_float_length = 6 # FIXME: check max length!!!
        if useRTInterface:
            self.rtmon.start()

        self.secmon.wait() # make sure we get data to not suprise clients
        

    def __repr__(self):
        return "Robot Object (IP=%s, state=%s)" % (self.host, self.secmon.get_all_data()["RobotModeData"])

    def __str__(self):
        return self.__repr__()

    def is_running(self): # legacy
        return self.secmon.running

    def is_program_running(self):
        return self.secmon.is_program_running()

    def send_program(self, prog):
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

    def set_tcp(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        """
        """
        if type(x) in (list, tuple):
            if len(x) != 6:
                raise Exception("Tcp is a 6 values list")
            else:
                arg = x
        else:
            arg = (x, y, z, rx, ry, rz)
        prog = "set_tcp(p[%s, %s, %s, %s, %s, %s])" % arg
        self.secmon.send_program(prog)

    def set_payload(self, weight):
        """
        set payload in Kg
        """
        prog = "set_payload(%s)" % weight 
        self.secmon.send_program(prog)

    def set_gravity(self, vector):
        """
        set direction of gravity
        """
        prog = "set_gravity(%s)" % list(vector) 
        self.secmon.send_program(prog)

    def send_message(self, msg):
        """
        send message to the GUI log tab on the robot controller
        """
        prog = "textmsg(%s)" % msg 
        self.secmon.send_program(prog)

    def set_digital_out(self, output, val):
        """
        set digital output. val is a bool
        """
        if val in (True, 1):
            val = "True"
        else:
            val = "False"
        self.secmon.send_program('digital_out[%s]=%s' % (output, val))

    def get_analog_inputs(self):
        """
        get analog input 
        """
        data = self.secmon.get_all_data()
        return data["MasterBoardData"]["analogInput0"], data["MasterBoardData"]["analogInput1"]


    def get_analog_in(self, nb):
        """
        get analog input 
        """
        data = self.secmon.get_all_data()
        return data["MasterBoardData"]["analogInput" + str(nb)]

    def get_digital_in_bits(self):
        """
        get digital output
        """
        data = self.secmon.get_all_data()
        return data["MasterBoardData"]["digitalInputBits"]

    def get_digital_in(self, nb):
        """
        get digital output
        """
        data = self.secmon.get_all_data()
        val = data["MasterBoardData"]["digitalInputBits"]
        mask = 1 << nb
        if  (val & mask):
            return 1
        else:
            return 0

    def get_digital_out(self, val):
        """
        get digital output
        """
        data = self.secmon.get_all_data()
        output = data["MasterBoardData"]["digitalOutputBits"]
        mask = 1 << val
        if  (output & mask):
            return 1
        else:
            return 0

    def set_analog_out(self, output, val):
        """
        set analog output, val is a float
        """
        prog = "set_analog_output(%s, %s)" % (output, val) 
        self.secmon.send_program(prog)

    def set_tool_voltage(self, val):
        """
        set voltage to be delivered to the tool, val is 0, 12 or 24
        """
        prog = "set_tool_voltage(%s)" % (val) 
        self.secmon.send_program(prog)

    def movej(self, joints, acc=0.1, vel=0.05, wait=True, relative=False):
        """
        move in joint space
        """
        if relative:
            l = self.getj()
            joints = [v + l[i] for i, v in enumerate(joints)]
        joints = [round(j, self.max_float_length) for j in joints] 
        prog = "movej(%s, a=%s, v=%s)" % (list(joints), acc, vel)
        joints.append(acc)
        joints.append(vel)
        prog = "movej([{},{},{},{},{},{}], a={}, v={})".format(*joints)
        self.send_program(prog)
        if not wait:
            return None
        else:
            self.wait_for_move()
            return self.getj()

    def wait_for_move(self):
        time.sleep(0.2)# it is important to sleep since robot may takes a while to get into running state
        while True:
            if not self.is_running():
                raise RobotException("Robot stopped")
            jts = self.secmon.get_joint_data(wait=True)
            finished = True
            for i in range(0, 6):
                if abs(jts["q_actual%s"%i] - jts["q_target%s"%i]) > self.radialEpsilon:
                    finished = False
                    break
            if finished and not self.secmon.is_program_running():
                return
 
    def getj(self, wait=False):
        """
        get joints position
        """
        jts = self.secmon.get_joint_data(wait)
        return [jts["q_actual0"], jts["q_actual1"], jts["q_actual2"], jts["q_actual3"], jts["q_actual4"], jts["q_actual5"]]

    def movel(self, tpose, acc=0.01, vel=0.01, wait=True, relative=False):
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
        prog = "movel(p[{},{},{},{},{},{}], a={}, v={})".format(*tpose)
        self.send_program(prog)
        if not wait:
            return None
        else:
            self.wait_for_move()
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

    def movec(self, pose, pose_via, pose_to, acc, vel, wait=True):
        """
        Move Circular: Move to position (circular in tool-space)
        see UR documentation
        """
        pose = [round(i, self.max_float_length) for i in pose]
        pose_via = [round(i, self.max_float_length) for i in pose_via]
        pose_to = [round(i, self.max_float_length) for i in pose_to]
        prog = "movec(p%s, p%s, p%s, a=%s, v=%s)" % (pose, pose_via, pose_to, acc, vel)
        self.send_program(prog)
        if not wait:
            return None
        else:
            self.wait_for_move()
            return self.getl()

    def movels(self, pose_list, acc=0.01, vel=0.01 , radius=0.01, wait=True):
        """
        where pose_list is a list of pose. 
        several movel commands must be send as one program in order for radius blending to work.
        """
        #TODO: This is could easily be implemented in movel by detecting type of the joint variable
        # can be implemented by sending a complete urscript program calling several movel in a row with a radius
        header = "def myProg():\n"
        end = "end\n"
        template = "movel(p[{},{},{},{},{},{}], a={}, v={}, r={})\n"
        prog = header
        for idx, pose in enumerate(pose_list):
            pose = [round(i, self.max_float_length) for i in pose]
            pose.append(acc)
            pose.append(vel)
            if idx != (len(pose_list) -1 ):
                pose.append(radius)
            else:
                pose.append(0)
            prog += template.format(*pose)
        prog += end
        print(prog)
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

    def _eq(self, l1, l2):
        """
        robot joints precision is 0.01, donot give anything smaller!!!
        """
        for i in range(0, len(l1)):
            if abs(l1[i] -l2[i]) > self.jointEpsilon:
                return False
        return True

    def _eqpose(self, l1, l2):
        """
        epsilonl is for x,y,z
        epsilonr is for a,b,c
        robot joints precision is 0.01, do not give anything smaller!!!
        """
        for i in range(0, 3):
            if abs(l1[i] - l2[i]) > self.linearEpsilon:
                #print "param: ", i, "val: ", l1[i], "-", l2[i] , "=", abs(l1[i] -l2[i]),  " is not under ", self.linearEpsilon
                return False
        for i in range(3, 6):
            if abs(l1[i] - l2[i]) > self.radialEpsilon:
                #print "param: ", i, "val: ", l1[i], "-", l2[i] , "=", abs(l1[i] -l2[i]),  " is not under ", self.radialEpsilon
                return False
        return True

    def cleanup(self):
        self.logger.info("Closing sockets to robot")
        self.secmon.cleanup()
        if self.rtmon:
            self.rtmon.stop()

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



class Robot(URRobot):
    """
    Generic Python interface to an industrial robot.
    Compare to the URRobot class, this class adds the possibilty to work directly with matrices
    and includes support for calibrating the robot coordinate system
    """
    def __init__(self, host, useRTInterface=False, logLevel = logging.WARN, parserLogLevel=logging.WARN):
        URRobot.__init__(self, host, useRTInterface, logLevel=logLevel, parserLogLevel=parserLogLevel)
        self.default_linear_acceleration = 0.01
        self.default_linear_velocity = 0.01

        self.calibration = math3d.Transform() #identity
        self.inverse = self.calibration.inverse()
        self.tracker = None

    def set_tcp(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        if type(x) == math3d.Transform:
            x = x.pose_vector
        URRobot.set_tcp(self, x, y, z, rx, ry, rz)

    def set_calibration_matrix(self, matrix):
        self.calibration = matrix
        self.inverse = self.calibration.inverse()

    def orient(self, orient, acc=None, vel=None, wait=True):
        if type(orient) != math3d.Orientation:
            orient = math3d.Orientation(orient)
        trans = self.get_transform()
        trans.orient = orient
        self.apply_transform(trans, acc, vel, wait)

    def set_orientation(self, orient, acc=None, vel=None, wait=True):
        self.orient(orient, acc, vel, wait)

    def translate(self, vect, acc=None, vel=None, wait=True):
        trans = self.get_transform()
        trans.pos += math3d.Vector(vect)
        return self.apply_transform(trans, acc, vel, wait)

    def set_pos(self, vect, acc=None, vel=None, wait=True):
        trans = math3d.Transform(self.get_orientation(), math3d.Vector(vect))
        return self.apply_transform(trans, acc, vel, wait)

    def apply_transform(self, trans, acc=None, vel=None, wait=True):
        if not acc: 
            acc = self.default_linear_acceleration
        if not vel: 
            vel = self.default_linear_velocity
        t = self.calibration * trans
        pose = URRobot.movel(self, t.pose_vector, acc, vel, wait)
        if pose != None : #movel does not return anything when wait is False
            return self.inverse * math3d.Transform(pose)

    def add_transform_b(self, trans, acc=None, vel=None, wait=True):
        """
        Add transform expressed in base coordinate
        """
        pose = self.get_transform()
        return self.apply_transform(trans * pose, acc, vel, wait)

    def add_transform_t(self, trans, acc=None, vel=None, wait=True):
        """
        Add transform expressed in tool coordinate
        """
        pose = self.get_transform()
        return self.apply_transform(pose * trans, acc, vel, wait)

    def get_transform(self, wait=False):
        pose = URRobot.getl(self, wait)
        trans = self.inverse * math3d.Transform(pose) 
        return trans

    def get_orientation(self, wait=False):
        trans  = self.get_transform(wait)
        return trans.orient

    def get_pos(self, wait=False):
        trans  = self.get_transform(wait)
        return trans.pos

    def movel(self, pose, acc=None, vel=None, wait=True, relative=False):
        """
        move linear to given pose in base coordinate
        """
        t = math3d.Transform(pose)
        if relative:
            self.add_transform_b(t, acc, vel, wait)
        else:
            self.apply_transform(t, acc, vel, wait)

    def movel_t(self, pose, acc=None, vel=None, wait=True):
        """
        move linear to given pose in tool coordinate
        """
        t = math3d.Transform(pose)
        self.add_transform_t(t, acc, vel, wait)

    def getl(self, wait=False):
        t = self.get_transform(wait)
        return t.pose_vector

    def set_gravity(self, vector):
        if type(vector) == math3d.Vector:
            vector = vector.list
        return URRobot.set_gravity(self, vector)

    def get_tracker(self):
        """
        return an object able to track robot move for logging
        """
        t = tracker.Tracker(self.host)
        t.set_calibration_matrix(self.calibration)
        return t


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




