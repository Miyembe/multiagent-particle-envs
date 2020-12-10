import socket
import numpy as np
import pickle
import struct

def boolify(s):
    if s == 'True':
        return True
    elif s == 'False':
        return False
    raise ValueError("Boolify Value Error")

def autoconvert(s):
    for fn in (boolify, int, float):
            try: 
                return fn(s)
            except ValueError:
                pass
    return s

class SocketNumpyArray():
    def __init__(self):
        self.address = ''
        self.port = 0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.type = None  # server or client
        self.sep = ' '

    def initialize_sender(self, address, port):
        """
        :param address: host address of the socket e.g 'localhost' or your ip
        :type address: str
        :param port: port in which the socket should be intialized. e.g 4000
        :type port: int
        :return: None
        :rtype: None
        """


        self.address = address
        self. port = port
        self.socket.connect((self.address, self.port))
        self.data = b''
        self.payload_size = struct.calcsize("L")

    def send_numpy_array(self, np_array):
        """
        :param np_array: Numpy array to send to the listening socket
        :type np_array: ndarray
        :return: None
        :rtype: None
        """
        data = pickle.dumps(np_array)

        # Send message length first
        message_size = struct.pack("L", len(data))  ### CHANGED

        # Then data
        self.socket.sendall(message_size + data)

    def send_number(self, num):
        """
        :param number: number to send to the listening socket
        :type number: int, float whatever numbers
        :return: None
        :rtype: None
        """

        val = str(num) + self.sep
        self.socket.sendall(val.encode())

    def initalize_receiver(self, port):
        """
        :param port: port to listen
        :type port: int
        :return: numpy array
        :rtype: ndarray
        """
        self.address = ''
        self.port = port
        self.socket.bind((self.address, self.port))
        print('Socket bind complete')
        self.socket.listen(10)
        self.conn, addr = self.socket.accept()
        print('Socket now listening')
        self.payload_size = struct.calcsize("L")  ### CHANGED
        self.data = b''

    def receive_array(self):

        while len(self.data) < self.payload_size:
            self.data += self.conn.recv(4096)

        packed_msg_size = self.data[:self.payload_size]
        self.data = self.data[self.payload_size:]
        msg_size = struct.unpack("L", packed_msg_size)[0]

        # Retrieve all data based on message size
        while len(self.data) < msg_size:
            self.data += self.conn.recv(4096)

        frame_data = self.data[:msg_size]
        self.data = self.data[msg_size:]

        # Extract frame
        frame = pickle.loads(frame_data)
        return frame

    def receive_number(self):
        buf = ''
        while self.sep not in buf:
            buf += self.conn.recv(8).decode()
        num = autoconvert(buf)
        
        return num
        
    def return_to_client(self, np_array):
        data = pickle.dumps(np_array)
        # Send message length first
        message_size = struct.pack("L", len(data))  ### CHANGED

        # Then data
        self.conn.send(message_size + data)

    def receive_from_server(self):

        while len(self.data) < self.payload_size:
            self.data += self.socket.recv(4096)

        packed_msg_size = self.data[:self.payload_size]
        self.data = self.data[self.payload_size:]
        msg_size = struct.unpack("L", packed_msg_size)[0]

        # Retrieve all data based on message size
        while len(self.data) < msg_size:
            self.data += self.socket.recv(4096)

        frame_data = self.data[:msg_size]
        self.data = self.data[msg_size:]

        # Extract frame
        frame = pickle.loads(frame_data)
        return frame