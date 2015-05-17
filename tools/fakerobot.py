import socket
import threading
import socketserver
import time

class RequestHandler(socketserver.BaseRequestHandler):
    #def __init__(self, *args, **kwargs):
        #print(self, *args, **kwargs)
        #print("Got connection from {}".format( self.client_address[0]) )
        #socketserver.BaseRequestHandler.__init__(self, *args, **kwargs)

    def handle(self):
        while True:
            data = str(self.request.recv(1024), 'ascii')
            cur_thread = threading.current_thread()
            print("{} received {} from {}".format(cur_thread.name, data, self.client_address) )
            if data == "":
                return

        #when this methods returns, the connection to the client closes

    def setup(self):
        print("Got new connection from {}".format( self.client_address) )
        self.server.handlers.append(self)

    def finish(self):
        print("Connection from {} lost".format( self.client_address) )
        self.server.handlers.remove(self)

class Server(socketserver.ThreadingMixIn, socketserver.TCPServer):
    def init(self):
        """
        __init__ should not be overriden
        """
        self.handlers = []

    def close(self):
        for handler in self.handlers:
            handler.request.shutdown(socket.SHUT_RDWR)
            handler.request.close()
        self.shutdown()


class FakeRobot(object):

    def run(self):
        host = "localhost"
        port = 30002
        server = Server((host, port), RequestHandler)
        server.init()
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.daemon = True
        server_thread.start()
        print("Fake Universal robot running at ", host, port)
        try:
            f = open("packet.bin", "rb")
            packet = f.read()
            f.close()
            while True:
                time.sleep(0.09) #The real robot published data 10 times a second
                for handler in server.handlers:
                    handler.request.sendall(packet)
        finally:
            print("Shutting down server")
            server.close()


if __name__ == "__main__":
    r = FakeRobot()
    r.run()



