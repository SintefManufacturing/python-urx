import time
from multiprocessing import Process, Queue, Event


class Tracker(Process):
    def __init__(self):
        self._queue = Queue()
        Process.__init__(self, args=(self._queue,))
        self._stop = Event()
        self._finished = Event()

    def run(self):
        print("Running")
        while not self._stop.is_set():
            time.sleep(1)
        self._queue.put([1,2,3,4,5,"lkj"])
        self._finished.set()
        print("Closing")

    def start_acquisition(self):
        self.start()

    def stop_acquisition(self):
        self._stop.set()

    def get_result(self):
        self._stop.set()
        while not self._finished.is_set():
            time.sleep(0.1)
        return self._queue.get()

    def shutdown(self):
        self._stop.set() # just to make sure 
        self.join()


if __name__ == "__main__":
    p = Tracker()
    try:
        p.start_acquisition()
        time.sleep(3)
        p.stop_acquisition()
        a = p.get_result()
        print("Result is: ", a)
    finally:
        p.shutdown()


