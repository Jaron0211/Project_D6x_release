#server#
# -*- coding: utf-8 -*-

import socket
HOST = '192.168.50.157'
PORT = 8000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(10)

while True:
    conn, addr = server.accept()
    clientMessage = str(conn.recv(1024), encoding='utf-8')

    print('\nClient message is: \n', clientMessage)

#    serverMessage =
#    conn.sendall(serverMessage.encode())
    conn.close()

#client#
'''
import socket

HOST = '192.168.50.157'
PORT = 8000
clientMessage = 'Hello!'

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST, PORT))
client.sendall(clientMessage.encode())

serverMessage = str(client.recv(1024), encoding='utf-8')
print('Server:', serverMessage)

client.close()
'''
