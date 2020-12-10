#!/usr/bin/env python
import numpy as np
import time
import socket

from make_env import make_env
from npsocket_sn import SocketNumpyArray
env = make_env('simple_two')


# Env initialisation
num_robots = env.n
action = np.asarray([[10.0,1.0], [1.0,1.0]])
#action = np.asarray([[1,1.1,1.2,4,5], [1,6,7,0.6,0.6]])

print(action.shape)
env.reset()
# Changing the sensitivity
env.agents[0].accel = 0.1 
env.agents[1].accel = 0.2
env.discrete_action_space = False
print(env.action_space)
print("force: {}".format(env.force_discrete_action))
# Communication Initialisation
# sock_sender = SocketNumpyArray()
# sock_sender.initialize_sender('localhost', 9999)

for i in range(100000):
    env.render()
    print("Action Space: {}".format(env.action_space))
    ids, obs, rewards, dones, infos = env.step(action)
    print("ids: {}, obs: {}, rewards: {}, dones: {}, infos: {}".format(ids, obs, rewards, dones, infos))
    # Publish the positions of robots
    # positions = np.asarray([agent.state.p_pos for agent in env.agents])
    # print(positions)
    # print(np.shape(positions))
    # start_time = time.time()
    # sock_sender.send_numpy_array(positions)
    # data = sock_sender.receive_from_server() 
    # end_time = time.time()
    # time_diff = end_time - start_time
    # print("Data received from receiver: {}, size: {}".format(data, data.shape))
    # print("Time delay: {}".format(time_diff))
    time.sleep(0.1)
    print("Step: {}".format(i))

#print(action)

#print("Wow")