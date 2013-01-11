import time
from multiprocessing import Process, Queue, Event


class Tracker(Process):
    def __init__(self):
        self._queue = Queue()
        Process.__init__(self, args=(self._queue,))
        self._quit = Event()
        self._quit.clear()

    def run(self):
        print("Running")
        while not self._quit.is_set():
            time.sleep(1)
        print("Closing")

    def get_result(self):
        pass

    def shutdown(self):
        self._quit.set()


if __name__ == "__main__":
    p = Tracker()
    try:
        p.start()
        time.sleep(3)
        p.shutdown()
    finally:
        p.join()


