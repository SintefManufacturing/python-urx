
import logging
import time
import socket
import re
SOCKET_PORT = 29999

__author__ = "Byeongdu Lee, <blee@anl.gov>, Argonne National Laboratory"
__license__ = "LGPLv3"

class dashboard(object):

    def __init__(self,
                 robot):
        self.robot = robot
        self.logger = logging.getLogger(u"dashboard")
        self.buff_size = 1024
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((robot.IP, SOCKET_PORT))
        reply = self._receive_ascii_bytes(self._socket)
        assert reply.startswith("Connected")

    def _receive_ascii_bytes(self, socket):
        reply = socket.recv(self.buff_size)
        #self.logger.info(f"receiving bytes:{reply}")
        reply_ascii = reply.decode(encoding="ascii")
        #self.logger.info(f"receiving:{reply_ascii}")
        return reply_ascii

    def _send_ascii_bytes(self, msg, socket):
        to_send = bytes(msg, encoding="ASCII")
        #self.logger.info(f"sending:{to_send}")
        socket.sendall(to_send)
        #self.logger.info("sent.")

    def __del__(self):
        # Should it stop recording here???
        # TODO : When deleting the object, make sure that only connected sockets will be closed.
        self._socket.shutdown(socket.SHUT_RDWR)
        self._socket.close()

    def _ensure_ready(self):
        reply = "Program running: true"
        while not reply.startswith("Program running: false"):
            time.sleep(0.12)  # seconds
            self._send_ascii_bytes("running\n", self._socket)
            reply = self._receive_ascii_bytes(self._socket)
        assert reply.startswith("Program running: false"), reply

    def send_program(self, program_name):
        self._send_ascii_bytes(f"{program_name}\n", self._socket)
        loaded_program_response = self._socket.recv(self.buff_size).decode()
        return loaded_program_response
    
    def get_loaded_program(self):
        self._send_ascii_bytes(f"get loaded program\n", self._socket)
        loaded_program = self._socket.recv(self.buff_size).decode()
        return loaded_program
    
    def unlock(self):
        val = self.send_program('unlock protective stop')
        if "Protective stop releasing" in val:
            return True
        else:
            return False

    def get_status(self):
        val = self.send_program('safetystatus')
        r = re.search('Safetystatus: (.*)\n', val)
        if hasattr(r, 'group'):
            return r.group(1)
        else:
            return False