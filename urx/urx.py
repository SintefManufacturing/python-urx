"""
Python library to control an UR robot through its TCP/IP interface

import urx

rob = urx.robot(192.168.0.100)
rob.set_tcp((x=01, z=0.232))
rob.movej((1,2,3,4,5,6), a, v)
rob.movel((x,y,z,a,b,c), a, v)
print "Current tool pose is: ",  rob.getl()
rob.movelrel((0.1, 0, 0, 0, 0, 0), a, v)
rob.stopj(a)

robot.movel(x,y,z,a,b,c), wait=False)
while True :
    sleep(0.1) #sleep first since the information may be outdated
    if robot.isProgramRunning():
        break

robot.movel(x,y,z,a,b,c), wait=False)
while.robot.getForce() < 50:
    sleep(0.01)
robot.stopl()

try:
    robot.movelrel(0,0,0.1,0,0,0)
except RobotError, ex:
    print "Robot could not execute move (emergency stop for example), do somethhing", ex

Using matrices:

robot = Robot("192.168.1.1")
robot.set_tcp((0,0,0.23,0,0,0)
calib = mathd3d.Transform() 
calib.orient.rotate_zb(pi/4) #just an example
robot.set_calibration_matrix(calib)

trans = robot.get_transform() # get current transformation matrix (tool to base)
trans.orient.rotate_yt(pi/2)
robot.apply_transform(trans)
trans.pos += math3d.Vector(0,0,0.3)
robot.apply_transform(trans)


#or only work with orientation part
o = robot.get_orientation()
o.rotate_yb(pi)
robot.orient(o)

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
    The RT interfaces is only used for the getForce related methods
    Rmq: A program sent to the robot i executed immendiatly and any running program is stopped
    """
    def __init__(self, host, useRTInterface=False, logLevel=logging.WARN):
        self.logger = logging.getLogger(self.__class__.__name__)
        if len(logging.root.handlers) == 0: #dirty hack
            logging.basicConfig()
        self.logger.setLevel(logLevel)
        self.host = host
        
        self.logger.info("Opening secondary monitor socket")
        self.secmon = ursecmon.SecondaryMonitor(self.host, logLevel=logLevel) #data from robot at 10Hz
        
        if useRTInterface:
            self.logger.info("Opening real-time monitor socket")
            self.rtmon = urrtmon.URRTMonitor(self.host)# som information is only available on rt interface
        else:
            self.rtmon = None
        #the next 3 values must be conservative! otherwise we may wait forever
        self.jointEpsilon = 0.05 # precision of joint movem used to wait for move completion
        self.linearEpsilon = 0.0005 # precision of linear movement, used to wait for move completion
        self.radialEpsilon = 0.05 # precision of radial movement, used to wait for move completion
        if useRTInterface:
            self.rtmon.start()

        self.start_pose = [1.57, -1.77, 1.57, -1.8, -1.57, -1.57]
        self.secmon.wait() # make sure we get data to not suprise clients
        

    def __repr__(self):
        return "Robot Object (IP=%s, state=%s)" % (self.host, self.secmon.getAllData()["RobotModeData"])

    def __str__(self):
        return self.__repr__()

    def isRunning(self): # legacy
        return self.secmon.running

    def isProgramRunning(self):
        return self.secmon.isProgramRunning()

    def sendProgram(self, prog):
        self.secmon.sendProgram(prog)

    def getTCPForce(self, wait=True):
        """
        return measured force in TCP
        if wait==True, waits for next packet before returning
        """
        return self.rtmon.getTCFForce(wait)

    def getForce(self, wait=True):
        """
        length of force vector returned by getTCPForce
        if wait==True, waits for next packet before returning
        """
        tcpf = self.getTCPForce( wait)
        force = 0
        for i in tcpf:
            force += i**2
        return force**0.5

    def moveToStartPose(self):
        """
        move to pos defined in self.start_pose attribute
        """
        self.movej(self.start_pose)

    def setTcp(self, x=0, y=0, z=0, a=0, b=0, c=0):
        """
        """
        if type(x) in (list, tuple):
            if len(x) != 6:
                raise Exception("Tcp is a 6 values list")
            else:
                arg = x
        else:
            arg = (x, y, z, a, b, c)
        prog = "set_tcp(p[%s, %s, %s, %s, %s, %s])" % arg
        self.secmon.sendProgram(prog)

    def setPayload(self, weight):
        """
        set payload in Kg
        """
        prog = "set_payload(%s)" % weight 
        self.secmon.sendProgram(prog)

    def setGravity(self, vector):
        """
        set direction of gravity
        """
        prog = "set_gravity(%s)" % list(vector) 
        self.secmon.sendProgram(prog)

    def sendMessage(self, msg):
        """
        send message to the GUI log tab on the robot controller
        """
        prog = "textmsg(%s)" % msg 
        self.secmon.sendProgram(prog)

    def setDigitalOut(self, output, val):
        """
        set digital output. val is a bool
        """
        if val in (True, 1):
            val = "True"
        else:
            val = "False"
        self.secmon.sendProgram('digital_out[%s]=%s' % (output, val))

    def getAnalogInputs(self):
        """
        get analog input 
        """
        data = self.secmon.getAllData()
        return data["MasterBoardData"]["analogInput0"], data["MasterBoardData"]["analogInput1"]


    def getAnalogInput(self, nb):
        """
        get analog input 
        """
        data = self.secmon.getAllData()
        return data["MasterBoardData"]["analogInput" + str(nb)]

    def getDigitalInputBits(self):
        """
        get digital output
        """
        data = self.secmon.getAllData()
        return data["MasterBoardData"]["digitalInputBits"]

    def getDigitalInput(self, nb):
        """
        get digital output
        """
        data = self.secmon.getAllData()
        val = data["MasterBoardData"]["digitalInputBits"]
        mask = 1 << nb
        if  (val & mask):
            return 1
        else:
            return 0

    def getDigitalOutput(self, val):
        """
        get digital output
        """
        data = self.secmon.getAllData()
        output = data["MasterBoardData"]["digitalOutputBits"]
        mask = 1 << val
        if  (output & mask):
            return 1
        else:
            return 0

    def setAnalogOut(self, output, val):
        """
        set analog output, val is a float
        """
        prog = "set_analog_output(%s, %s)" % (output, val) 
        self.secmon.sendProgram(prog)

    def setToolVoltage(self, val):
        """
        set voltage to be delivered to the tool, val is 0, 12 or 24
        """
        prog = "set_tool_voltage(%s)" % (val) 
        self.secmon.sendProgram(prog)

    def movejrel(self, joints, acc=0.1, vel=0.05, wait=True):
        """
        relative joint move
        """
        self.movej(joints, acc, vel, wait, relative=True)

    def movej(self, joints, acc=0.1, vel=0.05, wait=True, relative=False):
        """
        move in joint space
        """
        if relative:
            l = self.getj()
            joints = [v + l[i] for i, v in enumerate(joints)]
        prog = "movej(%s, a=%s, v=%s)" % (list(joints), acc, vel)
        self.sendProgram(prog)
        if not wait:
            return None
        else:
            time.sleep(0.05)# it is important to sleep since robot may takes a while to get into running state
            while True:
                if not self.isRunning():
                    raise RobotException("Robot stopped")
                currentjoints = self.getj(wait=True)
                if self._eq(currentjoints, joints) and not self.secmon.isProgramRunning(): 
                    return currentjoints

    def getj(self, wait=False):
        """
        get joints position
        """
        jts = self.secmon.getJointData(wait)
        return [jts["q_actual0"], jts["q_actual1"], jts["q_actual2"], jts["q_actual3"], jts["q_actual4"], jts["q_actual5"]]

    
    def movelrel(self, tpose, acc=0.01, vel=0.01, wait=True):
        """
        relative linear move 
        """
        return self.movel(tpose, acc, vel, wait, relative=True)

    def movel(self, tpose, acc=0.01, vel=0.01, wait=True, relative=False):
        """
        linear move 
        """
        if relative:
            l = self.getl()
            tpose = [v + l[i] for i, v in enumerate(tpose)]
        tpose = [round(i, 2) for i in tpose]
        prog = "movel(p%s, a=%s, v=%s)" % (tpose, acc, vel)
        self.sendProgram(prog)
        if not wait:
            return None
        else:
            time.sleep(0.05)# it is important to sleep since robot may takes a while to get into running state
            while True:
                if not self.isRunning():
                    raise RobotException("Robot stopped")
                pose = self.getl(wait=True)
                if self._eqpose(pose, tpose) and not self.secmon.isProgramRunning(): 
                    return pose

    def getl(self, wait=False):
        """
        get TCP position
        """
        pose = self.secmon.getCartesianInfo(wait)
        if pose:
            pose =  [pose["X"], pose["Y"], pose["Z"], pose["Rx"], pose["Ry"], pose["Rz"]]
        self.logger.debug("Current pose from robot: " + str(pose))
        return pose

    def movels(self, joints, acc, vel , radius, wait=True):
        """
        where joints is a list of list. dvs: several movel commands must be send as one program in order for radius blending to work.
        This is could easily be implemented in movel by detecting type of the joint variable
        """
        # can be implemented by sending a complete urscript program calling several movel in a row with a radius
        raise NotImplementedError
    
    def stopl(self, acc = 0.5):
        self.sendProgram("stopl(%s)" % acc)

    def stopj(self, acc = 1.5):
        self.sendProgram("stopj(%s)" % acc)

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
            self.sendProgram("set robotmode freedrive")
        else:
            self.sendProgram("set robotmode run")

    def set_simulation(self, val):
        if val:
            self.sendProgram("set sim")
        else:
            self.sendProgram("set real")



class Robot(object):
    """
    Generic Python interface to an industrial robot.
    Compare to the URRobot class, this class adds the possibilty to work directly with matrices
    and includes support for calibrating the robot coordinate system
    and style portet to PEP 8 
    """
    def __init__(self, host, useRTInterface=False, logLevel = logging.WARN):
        self.robot = URRobot(host, useRTInterface, logLevel=logLevel)
        self.logger = logging.getLogger(self.__class__.__name__)
        if len(logging.root.handlers) == 0: #dirty hack
            logging.basicConfig()
        self.logger.setLevel(logLevel)
        self.default_linear_acceleration = 0.01
        self.default_linear_velocity = 0.01

        self.calibration = math3d.Transform() #identity
        self.inverse = self.calibration.inverse()
        self.tracker = None

    def set_tcp(self, tcp):
        if type(tcp) == math3d.Transform:
            tcp = tcp.pose_vector
        self.robot.setTcp(tcp)

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

    def stop(self):
        self.robot.stop()

    def apply_transform(self, trans, acc=None, vel=None, wait=True):
        if not acc: 
            acc = self.default_linear_acceleration
        if not vel: 
            vel = self.default_linear_velocity
        t = self.calibration * trans
        pose = self.robot.movel(t.pose_vector, acc, vel, wait)
        if pose: #movel does not return anything when wait is False
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
        pose = self.robot.getl(wait)
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

    def is_running(self):
        return self.robot.isRunning()

    def is_program_running(self):
        return self.robot.isProgramRunning()

    def set_payload(self, weight):
        return self.robot.setPayload(weight)

    def set_gravity(self, vector):
        if type(vector) == math3d.Vector:
            vector = vector.list
        return self.robot.setGravity(vector)

    def set_digital_out(self, output, val):
        return self.robot.setDigitalOut(output, val)

    def get_digital_out(self, nb):
        self.robot.getDigitalOutput(nb)

    def get_digital_in(self, nb):
        return self.robot.getDigitalInput(nb)

    def get_analog_in(self, nb):
        return self.robot.getAnalogInput(nb)

    def set_freedrive(self, val):
        self.robot.set_freedrive(val)

    def set_simulation(self, val):
        self.robot.set_simulation(val)
 
    def movej(self, joints, acc=0.1, vel=0.05, wait=True, relative=False):
        """
        wrapper around the movej command in URRobot
        """
        self.robot.movej(joints, acc, vel, wait, relative)

    def getj(self, wait=False):
        return self.robot.getj(wait)

    def cleanup(self):
        self.robot.cleanup()

    def get_tracker(self):
        """
        return an object able to track robot move for logging
        """
        t = tracker.Tracker(self.robot.host)
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




