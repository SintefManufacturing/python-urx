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

TODO:
    add more methods
    movec

DOC LINK
http://support.universal-robots.com/URRobot/RemoteAccess
"""

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2012, Olivier Roulet-Dubonnet"
__credits__ = ["Olivier Roulet-Dubonnet"]
__license__ = "GPLv3"
__version__ = "0.3"
__status__ = "Development"


from threading import Thread, Lock, Condition
import socket
import time


MATH3D = True
try:
    import math3d
except ImportError:
    MATH3D = False
    print("pymath3d library could not be found on this computer, disabling use of matrices")

from urx import urrtmon 
from urx import urparser
from urx import tracker


class RobotException(Exception):
    pass


class SecondaryMonitor(Thread):
    """
    Monitor data from secondry port and send programs to robot
    """
    def __init__(self, host):
        Thread.__init__(self)
        self._lock = Lock()
        self._s_secondary = None
        self._dict = {}
        self._dictLock = Lock()
        self.host = host
        secondary_port = 30002    # Secondary client interface on Universal Robots
        self._s_secondary = socket.create_connection((self.host, secondary_port), timeout=0.5)
        self._queue = []
        self._dataqueue = bytes()
        self._trystop = False # to stop thread
        self.running = False #True when robot is on and listening
        self._dataEvent = Condition()

        self.start()
        with self._dataEvent:
            self._dataEvent.wait() # make sure we got some data before someone calls us

    def sendProgram(self, prog):
        """
        send program to robot in URRobot format
        If another program is send while a program is running the first program is aborded. 
        """

        with self._lock:
            prog.strip()
            self._queue.append(prog.encode() + b"\n")
 

    def run(self):
        """
        check program execution status in the secondary client data packet we get from the robot 
        This interface uses only data from the secondary client interface (see UR doc)
        Only the last connected client is the primary client, 
        so this is not guaranted and we cannot rely on information only 
        send to the primary client(like program execution start ?!?!?)
        """

        while not self._trystop:
            with self._lock:
                if len(self._queue) > 0:
                    prog = self._queue.pop(0)
                    self._s_secondary.send(prog)
            
            data = self._get_data()
            try:
                with self._dictLock:
                    self._dict = urparser.parse(data)
            except urparser.ParsingException as ex:
                self._log("Error parsing data from urrobot: " + str(ex) )
                continue

            if "RobotModeData" not in self._dict:
                self._log( "Got a packet from robot without RobotModeData, strange ...")
                continue

            #print data["RobotModeData"]

            if self._dict["RobotModeData"]["robotMode"] == 0 \
                            and self._dict["RobotModeData"]["isRealRobotEnabled"] == True \
                            and self._dict["RobotModeData"]["isEmergencyStopped"] == False \
                            and self._dict["RobotModeData"]["isSecurityStopped"] == False \
                            and self._dict["RobotModeData"]["isRobotConnected"] == True \
                            and self._dict["RobotModeData"]["isPowerOnRobot"] == True:
                self.running = True
            else:
                if self.running == True:
                    print("Robot not running: ", self._dict["RobotModeData"])
                self.running = False
            with self._dataEvent:
                self._dataEvent.notifyAll()


    def _get_data(self):
        """
        returns something that looks like a packet, nothing is guaranted
        """
        while True:
            ans = urparser.find_first_packet(self._dataqueue[:])
            if ans:
                self._dataqueue = ans[1]
                return ans[0]
            else:
                tmp = self._s_secondary.recv(1024)
                self._dataqueue += tmp

    def getCartesianInfo(self):
        with self._dictLock:
            if "CartesianInfo" in self._dict:
                return self._dict["CartesianInfo"]
            else:
                return None

    def getAllData(self):
        """
        return last data obtained from robot in dictionnary format
        """
        with self._dictLock:
            return self._dict.copy()


    def getJointData(self):
        with self._dictLock:
            if "JointData" in self._dict:
                return self._dict["JointData"]
            else:
                return None

    def isProgramRunning(self):
        """
        return True if robot is executing a program
        Rmq: The refresh rate is only 10Hz so the information may be outdated
        """
        with self._dictLock:
            return self._dict["RobotModeData"]["isProgramRunning"]


    def cleanup(self):
        self._trystop = True
        self.join()
        if self._s_secondary:
            with self._lock:
                self._s_secondary.close()

    def _log(self, msg):
        print(self.__class__.__name__, ": ", msg)


class URRobot(object):
    """
    Python interface to socket interface of UR robot.
    programs are send to port 3002
    data is read from secondary interface(10Hz?) and real-time interface(125Hz) (called Matlab interface in documentation)
    Since parsing the RT interface uses som CPU, and does not support all robots versions, it is disabled by default
    The RT interfaces is only used for the getForce related methods
    Rmq: A program sent to the robot i executed immendiatly and any running program is stopped
    """
    def __init__(self, host, useRTInterface=False):
        self.host = host
       
        self.secmon = SecondaryMonitor(self.host) #data from robot at 10Hz
        
        if useRTInterface:
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

    def getDigitalOutputBits(self):
        """
        get digital output
        """
        data = self.secmon.getAllData()
        return data["MasterBoardData"]["digitalOutputBits"]

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
        l = self.getj()
        newl = [v + l[i] for i, v in enumerate(joints)]
        return self.movej(newl, acc, vel, wait )

    def movej(self, joints, acc=0.1, vel=0.05, wait=True):
        """
        wrapper around the movej command in URRobot
        """
        #todo: should check joints input and velocity
        prog = "movej(%s, a=%s, v=%s)" % (list(joints), acc, vel)
        self.sendProgram(prog)
        if not wait:
            return None
        else:
            while True:
                time.sleep(0.05)# it is important to sleep since robot may takes a while to get into running state
                if not self.isRunning():#FIXME add more tests
                    raise RobotException("Robot stopped")
                currentjoints = self.getj()
                if self._eq(currentjoints, joints) and not self.secmon.isProgramRunning(): 
                    return currentjoints

    def getj(self):
        """
        get joints position
        """
        jts = self.secmon.getJointData()
        return [jts["q_actual0"], jts["q_actual1"], jts["q_actual2"], jts["q_actual3"], jts["q_actual4"], jts["q_actual5"]]

    
    def movelrel(self, tpose, acc=0.01, vel=0.01, wait=True):
        """
        relative move in cartesian coordinate
        """
        l = self.getl()
        newl = [v + l[i] for i, v in enumerate(tpose)]
        #print "going to: ", newl
        return self.movel(newl, acc, vel, wait )

    def movel(self, tpose, acc=0.01, vel=0.01, wait=True):
        """
        move in tool corrdinate system
        wrapper around the movel command in URRobot
        """
        #todo: should check joints input and velocity
        tpose = [round(i, 2) for i in tpose]
        prog = "movel(p%s, a=%s, v=%s)" % (tpose, acc, vel)
        #print(prog)
        self.sendProgram(prog)
        if not wait:
            return None
        else:
            while True:
                time.sleep(0.05)# it is important to sleep since robot may takes a while to get into running state
                if not self.isRunning():#FIXME add more tests
                    raise RobotException("Robot stopped")
                pose = self.getl()
                if self._eqpose(pose, tpose) and not self.secmon.isProgramRunning(): 
                    return pose

    def getl(self):
        """
        get TCP position
        """
        pose = self.secmon.getCartesianInfo()
        if pose:
            return [pose["X"], pose["Y"], pose["Z"], pose["Rx"], pose["Ry"], pose["Rz"]]
        else:
            return None

    def movels(self, joints, acc, vel , radius, wait=True):
        """
        where joints is a list of list. dvs: several movel commands must be send as one program in order for radius blending to work.
        This is could easily be implemented in movel by detecting type of the joint variable
        """
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

    def _log(self, msg):
        print("urrobot: ", self.__class__.__name__, ": ", msg)

    def cleanup(self):
        self.secmon.cleanup()
        if self.rtmon:
            self.rtmon.stop()



class Robot(object):
    """
    Generic Python interface to an industrial robot.
    Compare to the URRobot class, this class adds the possibilty to work directly with matrices
    and includes support for calibrating the robot coordinate system
    and style portet to PEP 8 
    """
    def __init__(self, host, useRTInterface=False):
        self.robot = URRobot(host, useRTInterface)
        self.default_linear_acceleration = 0.01
        self.default_linear_velocity = 0.01

        self.calibration = math3d.Transform() #identity
        self.inverse = self.calibration.inverse()

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

    def add_transform(self, trans, acc=None, vel=None, wait=True):
        pose = self.get_transform()
        return self.apply_transform(trans * pose, acc, vel, wait)

    def get_transform(self):
        pose = self.robot.getl()
        trans = self.inverse * math3d.Transform(pose) 
        return trans

    def get_orientation(self):
        trans  = self.get_transform()
        return trans.orient

    def get_pos(self):
        trans  = self.get_transform()
        return trans.pos

    def movel(self, pose, acc=None, vel=None, wait=True):
        t = math3d.Transform(pose)
        self.apply_transform(self.inverse * t, acc, vel, wait)

    def movelrel(self, pose, acc=None, vel=None, wait=True):
        self.apply_transform(math3d.Transform(pose), acc, vel, wait)

    def movel_cnc(self, pose, acc=None, vel=None, wait=True):
        """
        One could implement a CNC like interface with ABC like euler angles
        Not implemented yet
        """
        pass

    def getl(self):
        t = self.get_transform()
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
        self.robot.setDigitalOut(output, val)

    def movej(self, joints, acc=0.1, vel=0.05, wait=True):
        """
        wrapper around the movej command in URRobot
        """
        self.robot.movej(joints, acc, vel, wait)

    def getj(self):
        return self.robot.getj()

    def movejrel(self, joints, acc=0.1, vel=0.05, wait=True):
        """
        relative joint move
        """
        self.robot.movejrel(joints, acc, vel, wait)
    
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
    try:
        #robot = Robot( '192.168.1.6')
        robot = Robot( '192.168.1.5')
        r = robot

        #robot = Robot( '192.168.1.2', useRTInterface=False)
        #robot = Robot( '192.168.1.216')
        #robot = Robot( '192.168.1.216')
        #p = r.sendProgram(open("examples/example.prog").read(), wait=False)

        from IPython.frontend.terminal.embed import InteractiveShellEmbed
        ipshell = InteractiveShellEmbed( banner1="\nStarting IPython shell, robot object is available\n")
        ipshell(local_ns=locals())

    finally:
        if "robot" in dir():
            robot.cleanup()




