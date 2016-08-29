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


import logging
import socket
import struct
import time
import threading
from copy import deepcopy

import numpy as np

import math3d as m3d


class URRTMonitor(threading.Thread):
    '''
    Documentation to Real Time Client interface can be found here: 
    http://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/remote-control-via-tcpip-16496/
    '''
    # Struct for revision of the UR controller giving 1060 bytes (132 double values) 
    rtstruct1060 = struct.Struct('>132d')

    # Struct for revision of the UR controller giving 692 bytes(86 double values)
    rtstruct692 = struct.Struct('>86d')

    # for revision of the UR controller giving 540 byte. Here TCP
    # pose is not included! (67 double values)
    rtstruct540 = struct.Struct('>67d')

    def __init__(self, urHost):
        threading.Thread.__init__(self)
        self.logger = logging.getLogger(self.__class__.__name__)
        self.daemon = True
        self.running = False
        self._stop_event = True
        self._dataEvent = threading.Condition()
        self._dataAccess = threading.Lock()
        self._rtSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._rtSock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._urHost = urHost
        # Package data variables
        self._timestamp = None
        self._ctrlTimestamp = None
        self._qActual = None
        self._qTarget  = None #Target joint positions
        self._qdtarget  = None #Target joint velocities
        self._qddtarget  = None #Target joint accelerations
        self._current_target  = None #Target joint currents
        self._moment_target  = None #Target joint moments (torques)
        self._actual_digital_input_bits = None

        self._tcp = None
        self._tcp_force = None
        self.__recvTime = 0
        self._last_ctrl_ts = 0
        #self._last_ts = 0
        self._buffering = False
        self._buffer_lock = threading.Lock()
        self._buffer = []
        self._csys = None
        self._csys_lock = threading.Lock()
        self.aa = 0

    def set_csys(self, csys):
        with self._csys_lock:
            self._csys = csys

    def __recv_bytes(self, nBytes):
        ''' Facility method for receiving exactly "nBytes" bytes from
        the robot connector socket.'''
        # Record the time of arrival of the first of the stream block
        recvTime = 0
        pkg = b''
        while len(pkg) < nBytes:
            pkg += self._rtSock.recv(nBytes - len(pkg))
            if recvTime == 0:
                recvTime = time.time()
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

    def tcf_pose(self, wait=False, timestamp=False, ctrlTimestamp=False):
        """ Return the tool pose values."""
        if wait:
            self.wait()
        with self._dataAccess:
            tcf = self._tcp
            if ctrlTimestamp or timestamp:
                ret = [tcf]
                if timestamp:
                    ret.insert(-1, self._timestamp)
                if ctrlTimestamp:
                    ret.insert(-1, self._ctrlTimestamp)
                return ret
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
        # Record the timestamp for this logical package
        timestamp = self.__recvTime
        pkgsize = struct.unpack('>i', head)[0]
        self.logger.debug(
            'Received header telling that package is %s bytes long', 
            pkgsize)
        payload = self.__recv_bytes(pkgsize - 4)
        if pkgsize >= 1060:
            unp = self.rtstruct1060.unpack(payload[:self.rtstruct1060.size])
        elif pkgsize >= 692:
            unp = self.rtstruct692.unpack(payload[:self.rtstruct692.size])
        elif pkgsize >= 540:
            unp = self.rtstruct540.unpack(payload[:self.rtstruct540.size])
        else:
            self.logger.warning(
                'Error, Received packet of length smaller than 540: %s ',
                pkgsize)
            return

        with self._dataAccess:
            self._timestamp = timestamp
            # it seems that packet often arrives packed as two... maybe TCP_NODELAY is not set on UR controller??
            # if (self._timestamp - self._last_ts) > 0.010:
            #self.logger.warning("Error the we did not receive a packet for {}s ".format( self._timestamp - self._last_ts))
            #self._last_ts = self._timestamp
            self._ctrlTimestamp = np.array(unp[0]) #Time elapsed since the controller was started [s]
            if self._last_ctrl_ts != 0 and (
                    self._ctrlTimestamp -
                    self._last_ctrl_ts) > 0.010:
                self.logger.warning(
                    "Error the controller failed to send us a packet: time since last packet %s s ", 
                    self._ctrlTimestamp - self._last_ctrl_ts)
            self._last_ctrl_ts = self._ctrlTimestamp
            self._qTarget = np.array(unp[1:7]) #Target joint positions
            self._qdtarget = np.array(unp[7:13]) #Target joint velocities
            self._qddtarget = np.array(unp[13:19]) #Target joint accelerations
            self._current_target = np.array(unp[19:25]) #Target joint currents
            self._moment_target = np.array(unp[25:31]) #Target joint moments (torques)
            self._qActual = np.array(unp[31:37]) # Actual joint positions
            self._qdactual = np.array(unp[37:43]) #Actual joint velocities
            self._current_actual = np.array(unp[43:49]) #Actual joint currents
            self._joint_control_output = np.array(unp[49:55]) #Joint control currents
            self._actual_TCP_pose = np.array(unp[55:61]) #Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
            self._actual_TCP_speed = np.array(unp[61:67]) #Actual speed of the tool given in Cartesian coordinates
            self._tcp_force = np.array(unp[67:73]) #Generalized forces in the TCP
            self._tcp = np.array(unp[73:79]) #Target Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
            self._target_TCP_speed = np.array(unp[79:85]) #Target speed of the tool given in Cartesian coordinates
            self._actual_digital_input_bits = unp[85] #Current state of the digital inputs.
            self._joint_temperatures = np.array(unp[86:92]) #Temperature of each joint in degrees Celsius
            self._actual_execution_time = unp[92] #Controller real-time thread execution time
            self._URTest1 = unp[93] #A value used by Universal Robots software only                                                    
            self._robot_mode = unp[94] #Robot mode
            self._joint_mode = np.array(unp[95:101]) #Joint control modes
            self._safety_mode = unp[101] #Safety mode
            self._URTest2 = unp[102:108] #Used by Universal Robots software only                                                                                                        
            self._actual_tool_accelerometer = np.array(unp[108:111]) #Tool x, y and z accelerometer values
            self._URTest3 = unp[111:117] #Used by Universal Robots software only                                                                                                        
            self._speed_scaling = unp[117] #Speed scaling of the trajectory limiter
            self._actual_momentum = unp[118] #Norm of Cartesian linear momentum
            self._URTest4 = unp[119] #Used by Universal Robots software only                                                                                                        
            self._URTest5 = unp[120] #Used by Universal Robots software only                                                                                                        
            self._actual_main_voltagee = unp[121] #Masterboard: Main voltage
            self._actual_robot_voltage = unp[122] #Masterboard: Robot voltage (48V)
            self._actual_robot_current = unp[123] #Masterboard: Robot current
            self._actual_joint_voltage = np.array(unp[124:130]) #Actual joint voltages
            self._actual_digital_output_bits = unp[130] #Digital outputs
            self._Program_state = unp[131] #Program state
            
            if self._csys:
                with self._csys_lock:
                    # might be a godd idea to remove dependancy on m3d
                    tcp = self._csys * m3d.Transform(self._tcp)
                self._tcp = tcp.pose_vector
        if self._buffering:
            with self._buffer_lock:
                self._buffer.append(
                    (self._timestamp,
                     self._ctrlTimestamp,
                     self._tcp,
                     self._qActual))  # FIXME use named arrays of allow to configure what data to buffer

        with self._dataEvent:
            self._dataEvent.notifyAll()

    def start_buffering(self):
        """
        start buffering all data from controller
        """
        self._buffer = []
        self._buffering = True

    def stop_buffering(self):
        self._buffering = False

    def try_pop_buffer(self):
        """
        return oldest value in buffer
        """
        with self._buffer_lock:
            if len(self._buffer) > 0:
                return self._buffer.pop(0)
            else:
                return None

    def pop_buffer(self):
        """
        return oldest value in buffer
        """
        while True:
            with self._buffer_lock:
                if len(self._buffer) > 0:
                    return self._buffer.pop(0)
            time.sleep(0.001)

    def get_buffer(self):
        """
        return a copy of the entire buffer
        """
        with self._buffer_lock:
            return deepcopy(self._buffer)

    def get_all_data(self, wait=True):
        """
        return all data parsed from robot as a dict
        """
        if wait:
            self.wait()
        with self._dataAccess:
            return dict(
                timestamp=self._timestamp,
                ctrltimestamp=self._ctrlTimestamp,
                qActual=self._qActual,
                qTarget=self._qTarget,
                tcp=self._tcp,
                tcp_force=self._tcp_force)

    def stop(self):
        #print(self.__class__.__name__+': Stopping')
        self._stop_event = True

    def close(self):
        self.stop()
        self.join()

    def is_running(self):
        '''
        Return True if Real Time Client (RT) interface is running
        '''
        return self.running        
    
    def run(self):
        try:
            self._stop_event = False
            self._rtSock.connect((self._urHost, 30003))
            while not self._stop_event:
                self.__recv_rt_data()
                self.running = True
            self._rtSock.close()
        except:
            if self.running:
                self.running = False
                self.logger.error("RTDE interface stopped running")
        
        self.running = False        
        with self._dataEvent:
            self._dataEvent.notifyAll()


class URRTlogger(URRTMonitor, threading.Thread):
 
    def __init__(self, rtmon):
        threading.Thread.__init__(self)
        self.dataLog = logging.getLogger("RTC_Data_Log")
        self._stop_event = True
        self.rtmon = rtmon
         
    def logdata(self):
        self.rtmon.wait()
#        self.dataLog.info('target_q;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._qTarget)
#         self.dataLog.info('target_qd;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._qdtarget)
#         self.dataLog.info('target_qdd;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._qddtarget)
#         self.dataLog.info('target_current;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._current_target)
#         self.dataLog.info('target_moment;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._moment_target)
#         self.dataLog.info('actual_q;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._qActual)
#         self.dataLog.info('actual_qd;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._qdactual)
#         self.dataLog.info('actual_current;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._current_actual)
#         self.dataLog.info('joint_control_output;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._joint_control_output)
#         self.dataLog.info('actual_TCP_pose;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._actual_TCP_pose)
#         self.dataLog.info('actual_TCP_speed;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._actual_TCP_speed)
#         self.dataLog.info('actual_TCP_force;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._tcp_force)
#         self.dataLog.info('target_TCP_speed;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._target_TCP_speed)
#         self.dataLog.info('actual_digital_input_bits;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._actual_digital_input_bits)
#         self.dataLog.info('joint_temperatures;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._joint_temperatures)
#         self.dataLog.info('actual_execution_time;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._actual_execution_time)
# #?        self.dataLog.info('robot_mode;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._robot_mode)
# #?        self.dataLog.info('joint_mode;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._joint_mode)
# #?        self.dataLog.info('safety_mode;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._safety_mode)
# #?        self.dataLog.info('actual_tool_accelerometer;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._actual_tool_accelerometer)
#         self.dataLog.info('speed_scaling;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._speed_scaling)
#         self.dataLog.info('target_speed_fraction;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._target_speed_fraction)
#         self.dataLog.info('actual_momentum;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._actual_momentum)
#         self.dataLog.info('actual_main_voltage;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._actual_main_voltagee)
#         self.dataLog.info('actual_robot_voltage;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._actual_robot_voltage)
#         self.dataLog.info('actual_robot_current;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._actual_robot_current)
#         self.dataLog.info('actual_joint_voltage;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._actual_joint_voltage)
#         self.dataLog.info('actual_digital_output_bits;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._actual_digital_output_bits)
#         self.dataLog.info('runtime_state;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._runtime_state)
#         self.dataLog.info('robot_status_bits;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._robot_status_bits)
#         self.dataLog.info('safety_status_bits;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._safety_status_bits)
#         self.dataLog.info('analog_io_types;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._analog_io_types)
#         self.dataLog.info('standard_analog_input0;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._standard_analog_input0)
#         self.dataLog.info('standard_analog_input1;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._standard_analog_input1)
#         self.dataLog.info('standard_analog_output0;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._standard_analog_output0)
#         self.dataLog.info('standard_analog_output1;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._standard_analog_output1)
#         self.dataLog.info('io_current;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._io_current)
#         self.dataLog.info('euromap67_input_bits;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._euromap67_input_bits)
#         self.dataLog.info('euromap67_output_bits;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._euromap67_output_bits)
#         self.dataLog.info('euromap67_24V_voltage;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._euromap67_24V_voltage)
#         self.dataLog.info('euromap67_24V_current;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._euromap67_24V_current)
#         self.dataLog.info('tool_mode;%s;%s', self.rtmon._ctrlTimestamp, self.rtmon._tool_mode)
#         self.dataLog.info('target_TCP_pose;%s;%s;%s;%s;%s;%s;%s', self.rtmon._ctrlTimestamp, *self.rtmon._tcp)
         
    def stop(self):
        self._stop_event = True
 
    def close(self):
        self.stop()
        self.join()
 
    def run(self):
        self._stop_event = False
        while not self._stop_event:
            self.logdata()
         