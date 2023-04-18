import socket
import msgpack

s = socket.socket()
host = socket.gethostname

port =  8183

s.bind((host,port))

s.listen(1)

with open("test.csv") as f:
    while True:
        c, addr = s.accept()
