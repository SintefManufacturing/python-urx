import logging
import time
import socketserver
import socket
from threading import Thread, Lock

import math3d as m3d
import numpy as np
from IPython import embed

import urx


prog = '''
def myprog():
    textmsg("Starting program")
    ret = socket_open("192.168.0.215", 10002)
    if ret:
        textmsg("socket conected")
    else:
        textmsg("Coul not open socket aborting")
        return 0
    end
    textmsg("socket conected")
    socket_send_string(get_joint_positions())
    while True:
        action = socket_read_string(prefix="<", suffix=">")
        textmsg("command received: ", action)
        if action == "speedl":
            flist = socket_read_ascii_float(9)
            textmsg("speedl:", flist)
            target = [flist[1], flist[2], flist[3], flist[4], flist[5], flist[6]]
            acc = flist[7]
            t_min = flist[8]
            speedl(target, a=acc, t=t_min)
        elif action == "movel":
            flist = socket_read_ascii_float(9)
            textmsg("movel:", flist)
            global target = [flist[1], flist[2], flist[3], flist[4], flist[5], flist[6]]
            global vel = flist[7]
            global acc = flist[8]
            global cmd = action
            movel(target, v=vel, a=acc)
        elif action == "stop":
            textmsg("Stop request received")
            break
        end
        socket_send_string(get_joint_positions())
    end
    socket_close()
    textmsg("End program")

end
'''


prog_ss = '''
ret = True
if not ret:
    textmsg("YESSSSSSSSS")
end

'''


class URScriptServer(Thread):
    """
    Run a socket server in robot controller that received and 
    runs command from a client.
    This allows to process command at a rate of XX
    compared to only 10Hz using the secondary port interface
    """
    def __init__(self, robot):
        Thread.__init__(self)
        self.lock = Lock()
        self.robot = robot
        self.server = None


    def run(self):

        class MyTCPHandler(socketserver.BaseRequestHandler):
            def handle(self):
                self.server.handle = self
                ts = time.time()
                while not self.server.stop_request:
                    cmd = None
                    with self.server.lock:
                        if self.server.cmd:
                            cmd = self.server.cmd
                            self.server.cmd = None
                    if cmd:
                        print("Sending: ", cmd)
                        self.request.sendall(cmd)
                        pose = self.request.recv(1024).strip()
                        print("Received pose: ", pose)
                        cmd = None
                        ts = time.time()
                    if (time.time() - ts) > 1:
                        #print("KEEP ALIVE")
                        self.request.sendall(b"<keepalive>")
                        pose = self.request.recv(1024).strip()
                        ts = time.time()
                    time.sleep(0.001)

        self.server = socketserver.TCPServer(("0.0.0.0", 10002), MyTCPHandler)
        self.server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.server.cmd = None
        self.server.lock = self.lock
        self.server.stop_request = False
        self.server.serve_forever()

    def stop(self):
        with self.lock:
            self.server.cmd = b'<stop>'
        time.sleep(0.5)
        self.server.stop_request = True
        self.server.shutdown()

    def _send(self, cmd, *args):
        floats = []
        for arg in args:
            if isinstance(arg, (list, tuple)):
                floats.extend(arg)
            else:
                floats.append(arg)
        string = "<{}>({})".format(cmd, floats)
        string.replace("[", "(")
        string.replace("]", ")")
        with self.lock:
            self.server.cmd = string.encode('utf-8')

    def speedl(self, velocities, acc, min_time):
        v = self.robot.csys.orient * m3d.Vector(velocities[:3])
        w = self.robot.csys.orient * m3d.Vector(velocities[3:])
        vels = np.concatenate((v.array, w.array))
        return self._send("speedl", vels, acc, min_time)


if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING)
    r = urx.Robot("192.168.0.90")
    r.stop()  # stop any running things
    ctrl = URScriptServer(r)
    try:
        ctrl.start()
        time.sleep(1)
        r.send_program(prog)
        embed()
    finally:
        ctrl.stop()
        r.close()

