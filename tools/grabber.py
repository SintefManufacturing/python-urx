import socket
import sys


if __name__ == "__main__":
    host, port = "localhost", 30002
    host, port = "192.168.1.8", 30002
    if len(sys.argv) > 1:
        host = sys.argv[1]

    sock = socket.create_connection((host, port))
    f = open("packets.bin", "wb")

    try:
        # Connect to server and send data
        for i in range(0, 20):
            data = sock.recv(1024)
            f.write(data)
            print("Got packet: ", i)
    finally:
        f.close()
        sock.close()

