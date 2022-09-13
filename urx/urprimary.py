"""
This is to read data from primary client.
The primary client will send additonal information than the secondary such as Version info and others.
https://www.universal-robots.com/articles/ur/interface-communication/remote-control-via-tcpip/
I made this read-only. If you need to send a program, use the secondary monitor.
"""


import logging
import socket
from .urmon_parser import ParserUtils 

__author__ = "Byeongdu Lee, <blee@anl.gov>, Argonne National Laboratory"
__license__ = "LGPLv3"


class PrimaryMonitor(object):

    def __init__(self, host):
        self.logger = logging.getLogger("primarymon")
        self._parser = ParserUtils()
        self._dict = {}
        self._dataqueue = bytes()
        self.host = host
        self.lastpacket_timestamp = 0
        self.connect()
        self._get_version()

    def connect(self):
        primary_port = 30011    # Primary client interface on Universal Robots
        self._s_primary = socket.create_connection((self.host, primary_port), timeout=2.0)

    def get_data(self):
        """
        get data from the primary client.
        """
        data = self._get_data()
        tmpdict = self._parser.parse(data)
        self._dict.update(tmpdict)

    def _get_data(self):
        while True:
            ans = self._parser.find_first_packet(self._dataqueue[:])
            # returns None, when the packet is not complete.
            if ans:
                self._dataqueue = ans[1]
                return ans[0]
            else:
                # When the length of packet is shorter than defined by the first packet.
                # read again to make it full.
                tmp = self._s_primary.recv(2048)
                self._dataqueue += tmp
        
    def close(self):
        if self._s_primary:
            self._s_primary.close()

    def _get_version(self):
        tmp = self._s_primary.recv(1024)
        tmpdict = self._parser.parse(tmp)
        self._dict = tmpdict
        if "VersionMessage" in self._dict:
            self._parser.version = (self._dict["VersionMessage"]['majorVersion'], self._dict["VersionMessage"]['minorVersion'])
        else:
            self._parser.version = (0,0)

    def get_cartesian_info(self, wait=False):
        self.get_data()
        if "CartesianInfo" in self._dict:
            return self._dict["CartesianInfo"]
        else:
            return None

    def get_joint_data(self, wait=False):
        self.get_data()
        if "JointData" in self._dict:
            return self._dict["JointData"]
        else:
            return None

    def get_digital_out(self, nb, wait=False):
        self.get_data()
        output = self._dict["MasterBoardData"]["digitalOutputBits"]
        mask = 1 << nb
        if output & mask:
            return 1
        else:
            return 0

    def get_digital_out_bits(self, wait=False):
        self.get_data()
        return self._dict["MasterBoardData"]["digitalOutputBits"]

    def get_digital_in(self, nb, wait=False):
        self.get_data()
        output = self._dict["MasterBoardData"]["digitalInputBits"]
        mask = 1 << nb
        if output & mask:
            return 1
        else:
            return 0

    def get_digital_in_bits(self, wait=False):
        self.get_data()
        return self._dict["MasterBoardData"]["digitalInputBits"]

    def get_analog_in(self, nb, wait=False):
        self.get_data()
        return self._dict["MasterBoardData"]["analogInput" + str(nb)]

    def get_analog_inputs(self, wait=False):
        self.get_data()
        return self._dict["MasterBoardData"]["analogInput0"], self._dict["MasterBoardData"]["analogInput1"]

    def is_program_running(self, wait=False):
        self.get_data()
        return self._dict["RobotModeData"]["isProgramRunning"]