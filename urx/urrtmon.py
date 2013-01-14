'''
Module for implementing a UR controller real-time monitor over socket port 30003.
Confer http://support.universal-robots.com/Technical/RealTimeClientInterface
Note: The packet lenght given in the web-page is 740. What is actually received from the controller is 692. It is assumed that the motor currents, the last group of 48 bytes, are not send.
Originally Written by Morten Lind
'''

__author__ = "Morten Lind, Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011, NTNU/SINTEF Raufoss Manufacturing AS"
__credits__ = ["Morten Lind, Olivier Roulet-Dubonnet"]
__license__ = "GPLv3"
__version__ = "0.1"
__status__ = "Development"



import os
import socket
import struct 
import time
import threading

import numpy as np

class URRTMonitor(threading.Thread):

    ## Struct for revision of the UR controller giving 692 bytes
    rtstruct692 = struct.Struct('>d6d6d6d6d6d6d6d6d18d6d6d6dQ')
    
    ## for revision of the UR controller giving 540 byte. Here TCP
    ## pose is not included!
    rtstruct540 = struct.Struct('>d6d6d6d6d6d6d6d6d18d')

    def __init__(self, urHost, debug=False):
        threading.Thread.__init__(self)
        self.daemon = True
        self._stop = True
        self._debug = debug
        self._dataEvent = threading.Condition()
        self._dataAccess = threading.Lock()
        self._rtSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._urHost = urHost
        ## Package data variables
        self._timestamp = None
        self._qActual = None
        self._qTarget = None
        self._tcp = None
        self._tcp_force = None 
        self.__recvTime = 0

    def __recv_bytes(self, nBytes):
        ''' Facility method for receiving exactly "nBytes" bytes from
        the robot connector socket.'''
        ## Record the time of arrival of the first of the stream block
        recvTime = 0
        pkg = ''
        while len(pkg) < nBytes:
            pkg += self._rtSock.recv(nBytes - len(pkg))
            if recvTime == 0:
                recvTime  = time.time()
        self.__recvTime = recvTime
        return pkg

    def wait(self):
        with self._dataEvent:
            self._dataEvent.wait()

    def q_actual(self, wait=False, timestamp=False):
        """ Get the actual joint position vector."""
        if wait:
            self.wait()
        with self._dataAccess:
            if timestamp:
                return self._timestamp, self._qActual
            else:
                return self._qActual
    getActual = q_actual
    
    def q_target(self, wait=False, timestamp=False):
        """ Get the target joint position vector."""
        if wait:
            self.wait()
        with self._dataAccess:
            if timestamp:
                return self._timestamp, self._qTarget
            else:
                return self._qTarget
    getTarget = q_target
    
    def tcf_vec(self, wait=False, timestamp=False):
        """ Compute the tool pose *6D-vector* based on the actual joint
        values."""
        tcfvec = np.zeros(6)
        tcf = self.tcf_pose(wait=wait, timestamp=False)
        tcfvec[:3] = tcf.pos.data
        tcfvec[3:] = tcf.orient.toRotationVector().data
        if timestamp:
            return self._timestamp, tcfvec
        else:
            return tcfvec
    getTCFVec = tcf_vec
    
    def tcf_pose(self, wait=False, timestamp=False):
        """ Compute the tool pose *Transform* based on the actual joint
        values."""
        if wait:
            self.wait()
        with self._dataAccess:
            tcf = self._tcp
            if timestamp:
                return self._timestamp, tcf
            else:
                return tcf
    getTCF = tcf_pose

    def tcf_force(self, wait=False, timestamp=False):
        """ Get the tool force. The returned tool force is a
        six-vector of three forces and three moments."""
        if wait:
            self.wait()
        with self._dataAccess:
            # tcf = self._fwkin(self._qActual)
            tcf_force = self._tcp_force
            if timestamp:
                return self._timestamp, tcf_force
            else:
                return tcf_force
    getTCFForce = tcf_force

    def __recv_rt_data(self):
        head = self.__recv_bytes(4)
        ## Record the timestamp for this logical package
        timestamp = self.__recvTime
        pkgsize = struct.unpack('>i', head)[0]
        if self._debug:
            print('Received header telling that package is %d bytes long'%pkgsize)
        payload = self.__recv_bytes(pkgsize-4)
        if pkgsize >= 692:
            unp = self.rtstruct692.unpack(payload[:self.rtstruct692.size])
        elif pkgsize >= 540:
            unp = self.rtstruct540.unpack(payload[:self.rtstruct540.size])
        else:
            print('Error, Received packet of length smaller than 540: ', pkgsize)
            return

        with self._dataAccess:
            self._timestamp = timestamp
            self._qActual = np.array(unp[31:37])
            self._qTarget = np.array(unp[1:7])
            self._tcp_force = np.array(unp[67:73])
            self._tcp = np.array(unp[73:79])
        with self._dataEvent:
            self._dataEvent.notifyAll()

    def get_all_data(self, wait=True):
        """
        return all data parsed from robot as dict
        The content of the dict may vary depending on version of parser and robot controller
        but their name should stay constant
        """
        if wait:
            self.wait()
        with self._dataAccess:
            return dict(timestamp=self._timestamp, qActual=self._qActual, qTarget=self._qTarget, tcp=self._tcp, tcp_force=self._tcp_force)

    def stop(self):
        #print(self.__class__.__name__+': Stopping')
        self._stop = True

    def cleanup(self):
        self.stop()
        self.join()

    def run(self):
        self._stop = False
        self._rtSock.connect((self._urHost, 30003))
        while not self._stop:
            self.__recv_rt_data()
        self._rtSock.close()




def startupInteractive():
    global urRTMon
    from optparse import OptionParser
    ## Require the urhost arg.
    parser = OptionParser()
    parser.add_option('--debug', action='store_true', default=False, dest='debug')
    parser.add_option('--start', action='store_true', default=False, dest='start')
    opts, args = parser.parse_args()
    if len(args) != 1:
        raise Exception('Must have argument with ur-host name or ip!')
    urHost = args[0]
    print('Connecting to UR Secondary socket inteface on "%s"'%urHost)
    # # Start the connectors
    urRTMon = URRTMonitor(urHost, debug=opts.debug)
    if opts.start:
        urRTMon.start()
    # # Register for hard shutdown
    import atexit
    atexit.register(urRTMon.stop)
    ## Drop to interactive
    pystartfile = os.path.expanduser('~/.pythonstartup')
    if os.path.isfile(pystartfile):
        execfile(pystartfile)
    import code
    code.interact(None, None, globals())

if __name__ == '__main__':
    startupInteractive()
