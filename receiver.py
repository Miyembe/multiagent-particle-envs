import numpy as np
from npsocket_sn import SocketNumpyArray
sock_receiver = SocketNumpyArray()
sock_receiver.initalize_receiver(9999) # the 9999 is the port you can change it with your own. 
while True:
    positions = sock_receiver.receive_array()  # Receiving the image as numpy array. 
    # Display
    print("Received positions: {}, Size: {}".format(positions, np.shape(positions)))
    positions_re = positions.reshape(4,1)
    sock_receiver.return_to_client(positions_re)
    