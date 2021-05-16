"""
This file contains 2 classes:
    - ParseUtils containing utilies to parse data from UR robot
    - SecondaryMonitor, a class opening a socket to the robot and with methods to
            access data and send programs to the robot
Both use data from the secondary port of the URRobot.
Only the last connected socket on 3001 is the primary client !!!!
So do not rely on it unless you know no other client is running (Hint the UR java interface is a client...)
http://support.universal-robots.com/Technical/PrimaryAndSecondaryClientInterface
"""


from threading import Thread, Condition, Lock
import logging
import socket
import time
from .urmon_parser import ParsingException, ParserUtils 

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2013, Sintef Raufoss Manufacturing"
__credits__ = ["Olivier Roulet-Dubonnet"]
__license__ = "LGPLv3"


class OutofReachException(Exception):
    def __init__(self, *args):
        Exception.__init__(self, *args)

class ProtectiveStopException(Exception):
    def __init__(self, *args):
        Exception.__init__(self, *args)

class Program(object):
    def __init__(self, prog):
        self.program = prog
        self.condition = Condition()

    def __str__(self):
        return "Program({})".format(self.program)
    __repr__ = __str__


class TimeoutException(Exception):

    def __init__(self, *args):
        Exception.__init__(self, *args)

class SecondaryMonitor(Thread):

    """
    Monitor data from secondary port and send programs to robot
    """

    def __init__(self, host):
        Thread.__init__(self)
        self.logger = logging.getLogger("ursecmon")
        self._parser = ParserUtils()
        self._dict = {}
        self._dictLock = Lock()
        self.host = host
        self.connect()
        self._get_version()
        self._prog_queue = []
        self._prog_queue_lock = Lock()
        self._dataqueue = bytes()
        self._trystop = False  # to stop thread
        self.running = False  # True when robot is on and listening
        self._dataEvent = Condition()
        self.lastpacket_timestamp = 0

        self.start()
        try:
            self.wait()  # make sure we got some data before someone calls us
        except Exception as ex:
            self.close()
            raise ex

    def _get_version(self):
        tmp = self._s_secondary.recv(1024)
        tmpdict = self._parser.parse(tmp)
        self._dict = tmpdict
        if "VersionMessage" in self._dict:
            self._parser.version = (self._dict["VersionMessage"]['majorVersion'], self._dict["VersionMessage"]['minorVersion'])
        else:
            self._parser.version = (0,0)

    def connect(self):
        secondary_port = 30002    # Secondary client interface on Universal Robots
        self._s_secondary = socket.create_connection((self.host, secondary_port), timeout=2.0)

    def send_program(self, prog):
        """
        send program to robot in URRobot format
        If another program is send while a program is running the first program is aborded.
        """
        prog.strip()
        self.logger.debug("Enqueueing program: %s", prog)
        if not isinstance(prog, bytes):
            prog = prog.encode()

        data = Program(prog + b"\n")
        with data.condition:
            with self._prog_queue_lock:
                self._prog_queue.append(data)
            data.condition.wait(timeout=2.0)
            self.logger.debug("program sent: %s", data)

    def run(self):
        """
        check program execution status in the secondary client data packet we get from the robot
        This interface uses only data from the secondary client interface (see UR doc)
        Only the last connected client is the primary client,
        so this is not guaranted and we cannot rely on information to the primary client.
        """
        while not self._trystop:
            with self._prog_queue_lock:
                if len(self._prog_queue) > 0:
                    data = self._prog_queue.pop(0)
                    self._s_secondary.send(data.program)
                    with data.condition:
                        data.condition.notify_all()

            data = self._get_data()
            try:
                tmpdict = self._parser.parse(data)
                with self._dictLock:
                    self._dict = tmpdict
            except ParsingException as ex:
                self.logger.warning("Error parsing one packet from urrobot: %s", ex)
                continue

            if "RobotModeData" not in self._dict:
                self.logger.warning("Got a packet from robot without RobotModeData, strange ...")
                continue

            self.lastpacket_timestamp = time.time()

            rmode = 0
            if self._parser.version >= (3, 0):
                rmode = 7
                if self._parser.version >= (5.8):
                    if self._dict["RobotModeData"]["robotMode"] == rmode \
                            and self._dict["RobotModeData"]["isRealRobotEnabled"] is True \
                            and self._dict["RobotModeData"]["isEmergencyStopped"] is False \
                            and self._dict["RobotModeData"]["isProtectiveStopped"] is False \
                            and self._dict["RobotModeData"]["isRealRobotConnected"] is True \
                            and self._dict["RobotModeData"]["isRobotPowerOn"] is True:
                        self.running = True
                else:
                    if self._dict["RobotModeData"]["robotMode"] == rmode \
                            and self._dict["RobotModeData"]["isRealRobotEnabled"] is True \
                            and self._dict["RobotModeData"]["isEmergencyStopped"] is False \
                            and self._dict["RobotModeData"]["isSecurityStopped"] is False \
                            and self._dict["RobotModeData"]["isRobotConnected"] is True \
                            and self._dict["RobotModeData"]["isPowerOnRobot"] is True:
                        self.running = True                    
            else:
                if self.running:
                    self.logger.error("Robot not running: " + str(self._dict["RobotModeData"]))
                self.running = False
            with self._dataEvent:
                # print("X: new data")
                self._dataEvent.notifyAll()

    def _get_data(self):
        """
        returns something that looks like a packet, nothing is guaranted
        """
        while True:
            # self.logger.debug("data queue size is: {}".format(len(self._dataqueue)))
            ans = self._parser.find_first_packet(self._dataqueue[:])
            if ans:
                self._dataqueue = ans[1]
                # self.logger.debug("found packet of size {}".format(len(ans[0])))
                return ans[0]
            else:
                # self.logger.debug("Could not find packet in received data")
                try:
                    tmp = self._s_secondary.recv(1024)
                except socket.timeout as e:
                    self._s_secondary.close()
                    time.sleep(1.0)
                    self.connect()
                    raise TimeoutException("Did not receive a valid data packet from robot in {}".format(e))
                self._dataqueue += tmp

    def wait(self, timeout=2.0):
        """
        wait for next data packet from robot
        """
        tstamp = self.lastpacket_timestamp
        with self._dataEvent:
            try:
                self._dataEvent.wait(timeout)
            except socket.timeout as e:
                self._s_secondary.close()
                time.sleep(1.0)
                self.connect()
                raise TimeoutException("Did not receive a valid data packet from robot in {}".format(e))
            if tstamp == self.lastpacket_timestamp:
                raise TimeoutException("Did not receive a valid data packet from robot in {}".format(timeout))
            if self._parser.version >= (5.8):
                isProtectiveStopped = self._dict["RobotModeData"]["isProtectiveStopped"]
            else:
                isProtectiveStopped = self._dict["RobotModeData"]["isSecurityStopped"]
            if isProtectiveStopped is True:
                raise ProtectiveStopException("Protective stopped")

    def get_cartesian_info(self, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            if "CartesianInfo" in self._dict:
                return self._dict["CartesianInfo"]
            else:
                return None

    def get_all_data(self, wait=False):
        """
        return last data obtained from robot in dictionnary format
        """
        if wait:
            self.wait()
        with self._dictLock:
            return self._dict.copy()

    def get_joint_data(self, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            if "JointData" in self._dict:
                return self._dict["JointData"]
            else:
                return None

    def get_digital_out(self, nb, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            output = self._dict["MasterBoardData"]["digitalOutputBits"]
        mask = 1 << nb
        if output & mask:
            return 1
        else:
            return 0

    def get_digital_out_bits(self, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            return self._dict["MasterBoardData"]["digitalOutputBits"]

    def get_digital_in(self, nb, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            output = self._dict["MasterBoardData"]["digitalInputBits"]
        mask = 1 << nb
        if output & mask:
            return 1
        else:
            return 0

    def get_digital_in_bits(self, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            return self._dict["MasterBoardData"]["digitalInputBits"]

    def get_analog_in(self, nb, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            return self._dict["MasterBoardData"]["analogInput" + str(nb)]

    def get_analog_inputs(self, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            return self._dict["MasterBoardData"]["analogInput0"], self._dict["MasterBoardData"]["analogInput1"]

    def is_program_running(self, wait=False):
        """
        return True if robot is executing a program
        Rmq: The refresh rate is only 10Hz so the information may be outdated
        """
        if wait:
            self.wait()
        with self._dictLock:
            return self._dict["RobotModeData"]["isProgramRunning"]

    def is_protective_stopped(self, wait=False):
        if wait:
            self.wait()
        with self._dictLock:
            if self._parser.version >= (5,8):
                val = self.secmon._dict["RobotModeData"]["isProtectiveStopped"]
            else:
                val = self.secmon._dict["RobotModeData"]["isSecurityStopped"]
            return val

    def close(self):
        self._trystop = True
        self.join()
        # with self._dataEvent: #wake up any thread that may be waiting for data before we close. Should we do that?
        # self._dataEvent.notifyAll()
        if self._s_secondary:
            with self._prog_queue_lock:
                self._s_secondary.close()
                