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
    
    stop = False
    cmd = ""
    arg = False
    vel = 0.1
    t_min = 1
    acc = 0.1

    thread rt_servo():
        textmsg("rt_servo thread start")
        while not stop:
            if cmd == "speedl":
                textmsg("speedl received in servo thread")
                cmd = ""  # disabled to compensate for network/cpu delay
                speedl(target, a=acc, t=t_min)
            elif cmd == "movel":
                # just fo testing
                cmd = ""
                movel(target, v=vel, a=acc)
            else:
                sync()
            end
        end
        stopj(1)
        textmsg("rt_servo thread end")
    end


    thread rt_comm():
        textmsg("rt_comm thread start")
        ret = socket_open("192.168.0.215", 10002)
        textmsg("socket conected")
        while not stop:
            flist = socket_read_ascii_float(9)
            action = flist[1]
            if action == 1:
                textmsg("servoj:")
                textmsg(flist)
                global cmd = "servoj"
            elif action == 2:
                textmsg("speedl:")
                textmsg(flist)
                global target = [flist[2], flist[3], flist[4], flist[5], flist[6], flist[7]]
                global acc = flist[8]
                global t_min = flist[9]
                global cmd = "speedl"
            elif action == 0 :
                textmsg("Thread loop received stop, stopping")
                global stop = True
                break
            end
            sync()
            socket_send_string(get_joint_positions())
        end
        socket_close()
        textmsg("rt_comm thread end")
    end

 
    textmsg("START PROG")
    rt_comm_thread = run rt_comm()
    sleep(0.01)
    rt_servo_thread = run rt_servo()
    sleep(0.01)
    join rt_comm_thread
    join rt_servo_thread
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
        self.ts = 0
        self._stop_request = False
        self._conn = None

    def start(self):
        Thread.start(self)
        self.robot.send_program(prog)
        while self._conn is None:
            time.sleep(0.1)

    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.bind(('0.0.0.0', 10002))
            sock.listen(1)
            self._conn, addr = sock.accept()
            with self._conn:
                print('Connected by', addr)
                while not self._stop_request:
                    with self._lock:
                        if (time.time() - self.ts) > 0.5:
                            print("sending timeout", time.time(), self.ts, time.time() - self.ts)
                            s = b"99.99"
                            for i in range(8):
                                s += b", 0.0"
                            self._conn.sendall(s)
                            ans = self._conn.recv(1024).strip()
                            self.ts = time.time()

    def stop(self):
        self._send(0.0)  # 0 mean stop
        time.sleep(0.5)
        self._stop_request = True
        self.join()

    def _send(self, *args):
        floats = []
        for arg in args:
            if isinstance(arg, np.ndarray):
                floats.extend(arg.tolist())
            elif isinstance(arg, (list, tuple)):
                floats.extend(arg)
            else:
                floats.append(arg)
        while len(floats) != 9:
            floats.append(0.0)
            
        string = str(tuple(floats))
        cmd = string.encode('utf-8')
        with self._lock:
            self._conn.sendall(cmd)
            pose = self._conn.recv(1024).strip()
            if not pose or pose == b"CLOSE":
                # looks like ur sends a close string sometimes
                return None
            pose = pose[1:-1]
            pose = pose.split(b",")
            self.ts = time.time()
            return [float(i) for i in pose]

    def speedl(self, velocities, acc, min_time):
        v = self.robot.csys.orient * m3d.Vector(velocities[:3])
        w = self.robot.csys.orient * m3d.Vector(velocities[3:])
        vels = np.concatenate((v.array, w.array))
        return self._send(2.0, vels, acc, min_time)
    
    def speedl_tool(self, velocities, acc, min_time):
        pose = self.get_pose()
        v = pose.orient * m3d.Vector(velocities[:3])
        w = pose.orient * m3d.Vector(velocities[3:])
        vels = np.concatenate((v.array, w.array))
        return self._send(2.0, vels, acc, min_time)

    def servoj(self, joints, min_time):
        return self._send(1.0, joints, acc)
 

if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING)
    r = urx.Robot("192.168.0.90")
    r.stop()  # stop any running things
    ctrl = URScriptServer(r)
    try:
        ctrl.start()
        st = time.time()
        for i in range(100):
            ctrl.speedl((-0.1, 0, 0, 0, 0, 0), 1, 1/40)
        print("FREKVENS", 100/ (time.time() - st))
        embed()
    finally:
        ctrl.stop()
        r.close()

