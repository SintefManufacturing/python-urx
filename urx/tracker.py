"""
tracks moves a UR robot over the UR real-time port
"""
__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2013, Sintef Raufoss Manufacturing"
__credits__ = ["Olivier Roulet-Dubonnet"]
__license__ = "GPLv3"
import time
from multiprocessing import Process, Queue, Event

import numpy as np

from urx import urrtmon

MATH3D = True
try:
    import math3d
except ImportError:
    MATH3D = False
    print("pymath3d library could not be found on this computer, disabling use of matrices")

class Tracker(Process):
    def __init__(self, robot_host):
        self.host = robot_host
        self._queue = Queue()
        Process.__init__(self, args=(self._queue,))
        self._stop_event = Event()
        self._finished = Event()
        self._data = []
        if MATH3D:
            self.calibration = math3d.Transform()
            self.inverse = self.calibration.inverse()

    def _log(self, *args):
        print(self.__class__.__name__, ": ".join([str(i) for i in args]))

    def set_csys(self, cal):
        if MATH3D:
            self.calibration = cal
            self.inverse = self.calibration.inverse()

    def _save_data(self):
        result = np.zeros(len(self._data), dtype=[ ('timestamp', 'float64') , ('pose', '6float64'), ('joints', '6float64') ])
        for idx, data in enumerate(self._data):
            if MATH3D:
                trf = self.inverse * math3d.Transform(data[1])
            else:
                trf = data[1]
            result[idx] = (data[0], trf.pose_vector, data[2] )
        self._queue.put(result)


    def run(self):
        self._data = []
        rtmon = urrtmon.URRTMonitor(self.host)
        rtmon.start()
        while not self._stop_event.is_set():
            timestamp, pose = rtmon.tcf_pose(wait=True, timestamp=True)
            joints = rtmon.q_actual(wait=False, timestamp=False)
            self._data.append((timestamp, pose, joints))
        self._save_data()
        self._finished.set()

    def start_acquisition(self):
        self.start()

    def stop_acquisition(self):
        self._stop_event.set()
    stop = stop_acquisition

    def get_result(self):
        self._stop_event.set()
        while not self._finished.is_set():
            time.sleep(0.01)
        return self._queue.get()


if __name__ == "__main__":
    p = Tracker()
    try:
        p.start()
        time.sleep(3)
        p.stop()
        a = p.get_result()
        print("Result is: ", a)
    finally:
        p.stop()


