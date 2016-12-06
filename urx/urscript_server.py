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
        if action == "speedl":
            flist = socket_read_ascii_float(9)
            textmsg("speedl:", flist)
            target = [flist[1], flist[2], flist[3], flist[4], flist[5], flist[6]]
            acc = flist[7]
            t_min = flist[8]
            textmsg("START speedl")
            speedl(target, a=acc, t=t_min)
            textmsg("STOP speedl")
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


class URScriptServer(Thread):
    """
    Run a socket server in robot controller that received and 
    runs command from a client.
    This allows to process command at a rate of XX
    compared to only 10Hz using the secondary port interface
    """
    def __init__(self, robot):
        Thread.__init__(self)
        self._lock = Lock()
        self.robot = robot
        self.server = None
        self.ts = time.time()
        self._stop_request = False
        self._conn = None

    def start(self):
        Thread.start(self)
        self.robot.send_program(prog)
        print("SLEEP")
        while self._conn is None:
            time.sleep(0.1)
        time.sleep(1)
        print("END SLEEP")

    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.bind(('0.0.0.0', 10002))
            sock.listen(1)
            self._conn, addr = sock.accept()
            with self._conn:
                print('Connected by', addr)
                pose = self._conn.recv(1024)
                self.ts = time.time()
                while not self._stop_request:
                    if (time.time() - self.ts) > 1:
                        with self._lock:
                            self._conn.sendall(b"<keepalive>")
                            pose = self._conn.recv(1024).strip()
                        self.ts = time.time()

    def stop(self):
        self._send("stop")
        time.sleep(0.5)
        self._stop_request = True
        self.join()

    def _send(self, cmd, *args):
        floats = []
        for arg in args:
            if isinstance(arg, np.ndarray):
                floats.extend(arg.tolist())
            elif isinstance(arg, (list, tuple)):
                floats.extend(arg)
            else:
                floats.append(arg)
        string = "<{}>".format(cmd)
        if floats:
            string += str(tuple(floats))
        cmd = string.encode('utf-8')
        print("SENDING", cmd)
        with self._lock:
            self._conn.sendall(cmd)
            return self._conn.recv(1024).strip()

    def speedl(self, velocities, acc, min_time):
        v = self.robot.csys.orient * m3d.Vector(velocities[:3])
        w = self.robot.csys.orient * m3d.Vector(velocities[3:])
        vels = np.concatenate((v.array, w.array))
        return self._send("speedl", vels, acc, min_time)
    
    def speedl_tool(self, velocities, acc, min_time):
        pose = self.get_pose()
        v = pose.orient * m3d.Vector(velocities[:3])
        w = pose.orient * m3d.Vector(velocities[3:])
        vels = np.concatenate((v.array, w.array))
        return self._send("speedl", vels, acc, min_time)


if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING)
    r = urx.Robot("192.168.0.90")
    r.stop()  # stop any running things
    ctrl = URScriptServer(r)
    try:
        ctrl.start()
        for i in range(30):
            ctrl.speedl((0.1, 0, 0, 0, 0, 0), 1, 2)
            time.sleep(0.01)
        embed()
    finally:
        ctrl.stop()
        r.close()

