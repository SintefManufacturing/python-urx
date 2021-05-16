import logging
import struct
from copy import copy
# Origianlly written by Olivier Roulet-Dubonnet
# Rewritten for version 5.8 by Byeongdu Lee, 2/15/2021
#  
__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2013, Sintef Raufoss Manufacturing"
__credits__ = ["Olivier Roulet-Dubonnet"]
__license__ = "LGPLv3"

class ParsingException(Exception):

    def __init__(self, *args):
        Exception.__init__(self, *args)

class ParserUtils(object):

    def __init__(self):
        self.logger = logging.getLogger("ursecmon")
        self.version = (0, 0)

    def parse(self, data):
        """
        parse a packet from the UR socket and return a dictionary with the data
        """
        allData = {}
        while data:
            psize, ptype, pdata, data = self.analyze_header(data)
                # psize : packet size
                # ptype : Messagetype
                # pdata : the header part of the data
                # data : the data except the header.
            self.logger.debug("We got packet with size %i and type %s" % (psize, ptype))
            if ptype == 16:
                allData["ClientData"] = self._get_data(pdata, "!iB", ("size", "type"))
                data = (pdata + data)[5:]  # This is the total size so we resend data to parser
            elif ptype == 0:
                # this parses RobotModeData for versions >=3.0 (i.e. 3.0)
                if self.version >= (5, 8):
                    allData['RobotModeData'] = self._get_data(pdata, "!IBQ???????BBddB", ("size", "type", "timestamp", "isRealRobotConnected", "isRealRobotEnabled", "isRobotPowerOn", "isEmergencyStopped", "isProtectiveStopped", "isProgramRunning", "isProgramPaused", "robotMode", "controlMode", "targetSpeedFraction", "speedScaling", "targetSpeedFractionLimit", "ur_private"))
                else:
                    if psize == 38:
                        self.version = (3, 0)
                        allData['RobotModeData'] = self._get_data(pdata, "!IBQ???????BBdd", ("size", "type", "timestamp", "isPhysicalRobotConnected", "isRealRobotEnabled", "isPowerOnRobot", "isEmergencyStopped", "isProtectiveStopped", "isProgramRunning", "isProgramPaused", "robotMode", "controlMode", "speedFraction", "speedScaling"))
                    elif psize == 46:  # It's 46 bytes in 3.2
                        self.version = (3, 2)
                        allData['RobotModeData'] = self._get_data(pdata, "!IBQ???????BBdd", ("size", "type", "timestamp", "isPhysicalRobotConnected", "isRealRobotEnabled", "isPowerOnRobot", "isEmergencyStopped", "isProtectiveStopped", "isProgramRunning", "isProgramPaused", "robotMode", "controlMode", "speedFraction", "speedScaling", "speedFractionLimit"))
                    elif psize == 47:  # It's 47 bytes in 3.5
                        self.version = (3, 5)
                        allData['RobotModeData'] = self._get_data(pdata, "!IBQ???????BBddB", ("size", "type", "timestamp", "isPhysicalRobotConnected", "isRealRobotEnabled", "isPowerOnRobot", "isEmergencyStopped", "isProtectiveStopped", "isProgramRunning", "isProgramPaused", "robotMode", "controlMode", "speedFraction", "speedScaling", "speedFractionLimit", "ur_private"))
                    else:
                        allData["RobotModeData"] = self._get_data(pdata, "!iBQ???????Bd", ("size", "type", "timestamp", "isPhysicalRobotConnected", "isRealRobotEnabled", "isPowerOnRobot", "isEmergencyStopped", "isProtectiveStopped", "isProgramRunning", "isProgramPaused", "robotMode", "speedFraction"))
            elif ptype == 1:
                tmpstr = ["size", "type"]
                for i in range(0, 6):
                    tmpstr += ["q_actual%s" % i, "q_target%s" % i, "qd_actual%s" % i, "I_actual%s" % i, "V_actual%s" % i, "T_motor%s" % i, "T_micro%s" % i, "jointMode%s" % i]

                allData["JointData"] = self._get_data(pdata, "!iB dddffffB dddffffB dddffffB dddffffB dddffffB dddffffB", tmpstr)
            elif ptype == 2:
                allData["ToolData"] = self._get_data(pdata, "iBbbddfBffB", ("size", "type", "analoginputRange2", "analoginputRange3", "analogInput2", "analogInput3", "toolVoltage48V", "toolOutputVoltage", "toolCurrent", "toolTemperature", "toolMode"))
            elif ptype == 3:
                if self.version >= (3, 0):
                    fmt = "iBiibbddbbddffffBBb"     # firmware >= 3.0
                else:
                    fmt = "iBhhbbddbbddffffBBb"     # firmware < 3.0
                allData["MasterBoardData"] = self._get_data(pdata, fmt, ("size", "type", "digitalInputBits", "digitalOutputBits", "analogInputRange0", "analogInputRange1", "analogInput0", "analogInput1", "analogInputDomain0", "analogInputDomain1", "analogOutput0", "analogOutput1", "masterBoardTemperature", "robotVoltage48V", "robotCurrent", "masterIOCurrent"))  # , "masterSafetyState" ,"masterOnOffState", "euromap67InterfaceInstalled"   ))
            elif ptype == 4:
                if self.version < (3, 2):
                    allData["CartesianInfo"] = self._get_data(pdata, "iBdddddd", ("size", "type", "X", "Y", "Z", "Rx", "Ry", "Rz"))
                else:
                    allData["CartesianInfo"] = self._get_data(pdata, "iBdddddddddddd", ("size", "type", "X", "Y", "Z", "Rx", "Ry", "Rz", "tcpOffsetX", "tcpOffsetY", "tcpOffsetZ", "tcpOffsetRx", "tcpOffsetRy", "tcpOffsetRz"))
            elif ptype == 5:
                allData["KinematicsInfo"] = self._get_data(pdata, "!iBLLLLLL dddddd dddddd dddddd dddddd L", ("size", "type",
                "cheksum0","cheksum1","cheksum2","cheksum3","cheksum4","cheksum5",
                "DHtheta0","DHtheta1","DHtheta2","DHtheta3","DHtheta4","DHtheta5",
                "DHa0","DHa1","DHa2","DHa3","DHa4","DHa5",
                "Dhd0","Dhd1","Dhd2","Dhd3","Dhd4","Dhd5",
                "Dhalpha0","Dhalpha1","Dhalpha2","Dhalpha3","Dhalpha4","Dhalpha5",
                "calibration_status"))
            elif ptype == 6:
                allData["ConfigurationData"] = self._get_data(pdata, "!iBdddddddddddd dddddddddddd ddddd dddddd dddddd dddddd dddddd llll", 
                ("size", "type", 
                "jointMinLimit0", "jointMaxLimit0", "jointMinLimit1", "jointMaxLimit1", "jointMinLimit2", "jointMaxLimit2", "jointMinLimit3", "jointMaxLimit3", 
                "jointMinLimit4", "jointMaxLimit4", "jointMinLimit5", "jointMaxLimit5",
                "jointMaxSpeed0", "jointMaxAcceleration0","jointMaxSpeed1", "jointMaxAcceleration1","jointMaxSpeed2", "jointMaxAcceleration2","jointMaxSpeed3", "jointMaxAcceleration3",
                "jointMaxSpeed4", "jointMaxAcceleration4","jointMaxSpeed5", "jointMaxAcceleration5", 
                "vJointDefault", "aJointDefault", "vToolDefault", "aToolDefault", "eqRadius",
                "DHa0","DHa1","DHa2","DHa3","DHa4","DHa5",
                "Dhd0","Dhd1","Dhd2","Dhd3","Dhd4","Dhd5",
                "DHalpha0","DHalpha1","DHalpha2","DHalpha3","DHalpha4","DHalpha5",
                "DHtheta0","DHtheta1","DHtheta2","DHtheta3","DHtheta4","DHtheta5",
                "masterboardVersion", "controllerBoxType", "robotType", "robotSubType"))
            elif ptype == 7 and self.version >= (3, 2):
                allData["ForceModeData"] = self._get_data(pdata, "!iBddddddd", ("size", "type", "x", "y", "z", "rx", "ry", "rz", "robotDexterity"))
            elif ptype == 8 and self.version >= (3, 2):
                allData["AdditionalInfo"] = self._get_data(pdata, "!iB??", ("size", "type", "teachButtonPressed", "teachButtonEnabled"))
            elif ptype == 9:
                continue  # This package has a length of 53 bytes. It is used internally by Universal Robots software only and should be skipped.
            elif ptype == 10: # and self.version >= (3, 2):
                continue  # This is safety data. Also used only internally.
            elif ptype == 11 and self.version >= (3, 2):
                allData["ToolCommunicationInfo"] = self._get_data(pdata, "!iB?lllff", ("size", "type", "toolCommunicationisEnabled", "baudRate", "parity", "stopBits", "RxIdleChars", "TxIdleChars"))
            elif ptype == 12 and self.version >= (3, 2):
                allData["ToolModeInfo"] = self._get_data(pdata, "!iBBBB", ("size", "type", "outputMode", "output0", "output1"))
            elif ptype == 20:
                # this is for version 5.8
                self.logger.info("ptype 20 is read.")
                tmp = self._get_data(pdata, "!iBQbb", ("size", "type", "timestamp", "source", "robotMessageType"))
                if tmp["robotMessageType"] == 3:
                    allData["VersionMessage"] = self._get_data(pdata, "!iBQbbbA BBiiA", ("size", "type", "timestamp", "source", "robotMessageType", "projectNameSize", "projectName", "majorVersion", "minorVersion", "bugfixVersion", "buildNumber", "buildDate"))
                elif tmp["robotMessageType"] == 6:
                    allData["robotCommMessage"] = self._get_data(pdata, "!iBQbbiiiLLA", ("size", "type", "timestamp", "source", "robotMessageType", "code", "argument", "level", "datatype", "data", "messageText"))
                elif tmp["robotMessageType"] == 2:
                    allData["popupMessage"] = self._get_data(pdata, "!iBQbbIB???HA A", ("size", "type", "timestamp", "source", "robotMessageType", "id", "type", "warning", "error", "blocking", "titleSize", "messageTitle", "messageText"))
                elif tmp["robotMessageType"] == 0:
                    allData["messageText"] = self._get_data(pdata, "!iBQbbbA", ("size", "type", "timestamp", "source", "robotMessageType", "messageText"))
                elif tmp["robotMessageType"] == 7:
                    allData["keyMessage"] = self._get_data(pdata, "!iBQbbiiHA A", ("size", "type", "timestamp", "source", "robotMessageType", "code", "argument", "titleSize", "messageTitle", "messageText"))
                elif tmp["robotMessageType"] == 5:
                    allData["SafetyMessage"] = self._get_data(pdata, "!iBQbbiiBII", ("size", "type", "timestamp", "source", "robotMessageType", "code", "argument", "Modetype", "Datatype", "Data"))
                else:
                    print(tmp["robotMessageType"], "messagetype is not impremetned yet.")
                    self.logger.debug("Message type parser not implemented %s", tmp)
            elif ptype == 25:
                tmp = self._get_data(pdata, "!iBQbb", ("size", "type", "timestamp", "source", "robotMessageType"))
                self.logger.info("ptype 25 is read.")
                if tmp["robotMessageType"] == 2:
                    allData["varMessage"] = self._get_data(pdata, "!iBQbHA A", ("size", "type", "timestamp", "robotMessageType","titleSize", "messageTitle", "messageText"))
                if tmp["robotMessageType"] == 0:
                    allData["globalVariablesSetupMessage"] = self._get_data(pdata, "!iBQbIA", ("size", "type", "timestamp", "robotMessageType","startIndex", "variableNames"))
                if tmp["robotMessageType"] == 1:
                    allData["globalVariablesUpdateMessage"] = self._get_data(pdata, "!iBQbIA", ("size", "type", "timestamp", "robotMessageType","startIndex", "variableValues"))
                # Each variable value contains type byte, data, and terminating new line character (\n character). Example - two variables of type BOOL(True), and STRING("aaaa"): 0c 01 0a 03 00 04 61 61 61 61 0a
            else:
                print("Unknown packet type %s with size %s" %(ptype, psize))
                #self.logger.debug("Unknown packet type %s with size %s", ptype, psize)
        return allData

    def _get_data(self, data, fmt, names):
        """
        fill data into a dictionary
            data is data from robot packet
            fmt is struct format, but with added A for arrays and no support for numerical in fmt
            names args are strings used to store values
        """
        tmpdata = copy(data)
        fmt = fmt.strip()  # space may confuse us
        d = dict()
        i = 0
        j = 0
        while j < len(fmt) and i < len(names):
            f = fmt[j]
            if f in (" ", "!", ">", "<"):
                j += 1
            elif f == "A":  # we got an array
                # first we need to find its size
                if j == len(fmt) - 1:  # we are last element, size is the rest of data in packet
                    arraysize = len(tmpdata)
                else:  # size should be given in last element
                    asn = names[i - 1]
                    if not asn.endswith("Size"):
                        raise ParsingException("Error, array without size ! %s %s" % (asn, i))
                    else:
                        arraysize = d[asn]
                d[names[i]] = tmpdata[0:arraysize]
                tmpdata = tmpdata[arraysize:]
                j += 2
                i += 1
            else:
                fmtsize = struct.calcsize(fmt[j])
                # print "reading ", f , i, j,  fmtsize, len(tmpdata)
                if len(tmpdata) < fmtsize:  # seems to happen on windows
                    raise ParsingException("Error, length of data smaller than advertized: ", len(tmpdata), fmtsize, "for names ", names, f, i, j)
                d[names[i]] = struct.unpack("!" + f, tmpdata[0:fmtsize])[0]
                # print names[i], d[names[i]]
#                print('%s is %s with size of %i' % (names[i], d[names[i]], fmtsize))
                tmpdata = tmpdata[fmtsize:]
                j += 1
                i += 1
        return d

    def get_header(self, data):
        return struct.unpack("!iB", data[0:5])

    def analyze_header(self, data):
        """
        read first 5 bytes and return complete packet
        """
        if len(data) < 5:
            raise ParsingException("Packet size %s smaller than header size (5 bytes)" % len(data))
        else:
            psize, ptype = self.get_header(data)
            if psize < 5:
                raise ParsingException("Error, declared length of data smaller than its own header(5): ", psize)
            elif psize > len(data):
                raise ParsingException("Error, length of data smaller (%s) than declared (%s)" % (len(data), psize))
        return psize, ptype, data[:psize], data[psize:]

    def find_first_packet(self, data):
        """
        find the first complete packet in a string
        returns None if none found
        """
        counter = 0
        limit = 10
        while True:
            if len(data) >= 5:
                psize, ptype = self.get_header(data)
                # if ptype == 20 and psize<2000:
                #     return (data[:psize], data[psize:])
                if psize < 5 or psize > 2000 or ptype != 16:
                    data = data[1:]
                    counter += 1
                    if counter > limit:
#                        self.logger.warning("tried %s times to find a packet in data, advertised packet size: %s, type: %s", counter, psize, ptype)
#                        self.logger.warning("Data length: %s", len(data))
                        limit = limit * 10
                elif len(data) >= psize:
                    self.logger.debug("Got packet with size %s and type %s" %(psize, ptype))
                    if counter:
                        self.logger.info("Remove %s bytes of garbage at begining of packet", counter)
                    # ok we we have somehting which looks like a packet"
                    return (data[:psize], data[psize:])
                else:
                    # packet is not complete
                    self.logger.debug("Packet is not complete, advertised size is %s, received size is %s, type is %s", psize, len(data), ptype)
                    return None
            else:
#                self.logger.info("data smaller than 5 bytes")
                return None