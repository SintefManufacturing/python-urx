"""
parse data from UR socket client
REMARK
Only the last connected socket on 3001 is the primary client !!!! so it is unreliable to rely on it
http://support.universal-robots.com/Technical/PrimaryAndSecondaryClientInterface
"""

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2012, Olivier Roulet-Dubonnet"
__credits__ = ["Olivier Roulet-Dubonnet"]
__license__ = "GPLv3"
__version__ = "0.3"
__status__ = "Development"


import struct 
from copy import copy

class ParsingException(Exception):
    def __init__(self, *args):
        Exception.__init__(self, *args)


def _get_data(data, fmt, names):
    """
    fill data into a dictionary
        data is data from robot packet
        fmt is struct format, but with added A for arrays and no support for numerical in fmt
        names args are strings used to store values
    """

    tmpdata = copy(data)
    fmt = fmt.strip() # space may confuse us
    d = dict()
    i = 0
    j = 0
    while j < len(fmt) and i < len(names):
        f = fmt[j]
        if f in (" ", "!", ">", "<"):
            j += 1
        elif f == "A": #we got an array 
            # first we need to find its size
            if j == len(fmt) - 2: # we are last element, size is the rest of data in packet
                arraysize = len(tmpdata)
            else: # size should be given in last element
                asn =  names[i-1]
                if not asn.endswith("Size"):
                    raise ParsingException("Error, array without size ! %s %s" % (asn, i))
                else:
                    arraysize = d[asn]
            d[names[i]] = tmpdata[0:arraysize]
            #print "Array is ", names[i], d[names[i]]
            tmpdata = tmpdata[arraysize:]
            j += 2
            i += 1
        else:
            fmtsize = struct.calcsize(fmt[j])
            #print "reading ", f , i, j,  fmtsize, len(tmpdata)
            if len(tmpdata) < fmtsize: #seems to happen on windows
                raise ParsingException("Error, length of data smaller than advertized: ", len(tmpdata), fmtsize, "for names ", names, f, i, j)
            d[names[i]] = struct.unpack("!" + f, tmpdata[0:fmtsize])[0]
            #print names[i], d[names[i]]
            tmpdata = tmpdata[fmtsize:]
            j += 1
            i += 1

    return d 

def get_header(data):
    return struct.unpack("!iB", data[0:5])

def analyze_header(data):
    """
    read first 5 bytes and return complete packet
    """
    if not len(data) >= 5:
        raise ParsingException("Packet size %s smaller than header size (5 bytes)" % len(data))
    else:
        psize, ptype = get_header(data)
        if psize < 5: 
            raise ParsingException("Error, declared length of data smaller than its own header(5): ", psize)
        elif psize > len(data):
            raise ParsingException("Error, length of data smaller (%s) than declared (%s)" %( len(data), psize))
    return psize, ptype, data[:psize], data[psize:]


def find_first_packet(data):
    """
    find the first complete packet in a string
    returns None if none found
    """
    counter = 0
    while True:
        if len(data) >= 5:
            psize, ptype = get_header(data)
            if psize < 5 or psize > 500 or ptype != 16:
                data = data[1:]
            elif len(data) > psize:
                if counter:
                    print("Had to remove % bytes of garbage at begining of packet" % counter)
                #ok we we have somehting which looks like a packet"
                return (data[:psize], data[psize:])
            else:
                #packet is not complete
                return None
        else:
            return None
    


def parse(data):
    """
    parse a packet from the UR socket and return a dictionary with the data
    """
    allData = {}
    #print "Total size ", len(data)
    while data:
        psize, ptype, pdata, data = analyze_header(data)
        #print "We got packet with size %i and type %s" % (psize, ptype)
        if ptype == 16:
            allData["SecondaryClientData"] = _get_data(pdata, "!iB", ("size", "type"))
            data = (pdata + data)[5:] # This is the total size so we resend data to parser
        elif ptype == 0:
            allData["RobotModeData"] = _get_data(pdata, "!iBQ???????Bd", ("size", "type", "timestamp", "isRobotConnected", "isRealRobotEnabled", "isPowerOnRobot", "isEmergencyStopped", "isSecurityStopped", "isProgramRunning", "isProgramPaused", "robotMode", "speedFraction"))
        elif ptype == 1:
            tmpstr = ["size", "type"]
            for i in range(0, 6):
                tmpstr += ["q_actual%s" % i, "q_target%s" % i, "qd_actual%s" % i, "I_actual%s" % i, "V_actual%s" % i, "T_motor%s" % i ,"T_micro%s" % i, "jointMode%s" % i]

            allData["JointData"] = _get_data(pdata, "!iB dddffffB dddffffB dddffffB dddffffB dddffffB dddffffB", tmpstr)

        elif ptype == 4:
            allData["CartesianInfo"] = _get_data(pdata, "iBdddddd", ("size", "type", "X", "Y", "Z", "Rx", "Ry", "Rz"))
        elif ptype == 5:
            allData["LaserPointer(OBSOLETE)"] = _get_data(pdata, "iBddd" , ("size", "type"))
        elif ptype == 3:
            allData["MasterBoardData"] = _get_data(pdata, "iBhhbbddbbddffffBBb" , ("size", "type", "digitalInputBits", "digitalOutputBits", "analogInputRange0", "analogInputRange1", "analogInput0", "analogInput1", "analogInputDomain0", "analogInputDomain1", "analogOutput0", "analogOutput1", "masterBoardTemperature", "robotVoltage48V", "robotCurrent", "masterIOCurrent"))#, "masterSafetyState" ,"masterOnOffState", "euromap67InterfaceInstalled"   ))
        elif ptype == 2:
            allData["ToolData"] = _get_data(pdata, "iBbbddfBffB" , ("size", "type", "analoginputRange2", "analoginputRange3", "analogInput2", "analogInput3", "toolVoltage48V", "toolOutputVoltage", "toolCurrent", "toolTemperature", "toolMode" ))

        elif ptype == 20:
            tmp = _get_data(pdata, "!iB Qbb", ("size", "type", "timestamp", "source", "robotMessageType"))
            if tmp["robotMessageType"] == 3:
                allData["VersionMessage"] = _get_data(pdata, "!iBQbb bAbBBiAb", ("size", "type", "timestamp", "source", "robotMessageType", "projectNameSize", "projectName", "majorVersion", "minorVersion", "svnRevision", "buildDate"))
            elif tmp["robotMessageType"] == 6:
                allData["robotCommMessage"] = _get_data(pdata, "!iBQbb iiAc", ("size", "type", "timestamp", "source", "robotMessageType", "code", "argument", "messageText"))
            elif tmp["robotMessageType"] == 1:
                allData["labelMessage"] = _get_data(pdata, "!iBQbb iAc", ("size", "type", "timestamp", "source", "robotMessageType", "id", "messageText"))
            elif tmp["robotMessageType"] == 2:
                allData["popupMessage"] = _get_data(pdata, "!iBQbb ??BAcAc", ("size", "type", "timestamp", "source", "robotMessageType", "warning", "error", "titleSize", "messageTitle", "messageText"))
            elif tmp["robotMessageType"] == 0:
                allData["messageText"] = _get_data(pdata, "!iBQbb Ac", ("size", "type", "timestamp", "source", "robotMessageType", "messageText"))
            elif tmp["robotMessageType"] == 8:
                allData["varMessage"] = _get_data(pdata, "!iBQbb iiBAcAc", ("size", "type", "timestamp", "source", "robotMessageType", "code", "argument", "titleSize", "messageTitle", "messageText"))
            elif tmp["robotMessageType"] == 7:
                allData["keyMessage"] = _get_data(pdata, "!iBQbb iiBAcAc", ("size", "type", "timestamp", "source", "robotMessageType", "code", "argument", "titleSize", "messageTitle", "messageText"))
            elif tmp["robotMessageType"] == 5:
                allData["keyMessage"] = _get_data(pdata, "!iBQbb iiAc", ("size", "type", "timestamp", "source", "robotMessageType", "code", "argument", "messageText"))
            else:
                print("Message type parser not implemented ", tmp)
        else:
            print("Uknown packet type %s with size %s" % (ptype, psize))

    return allData






