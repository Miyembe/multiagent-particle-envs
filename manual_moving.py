import numpy as np
from make_env import make_env
import time

# Initialising environment
env = make_env('experiment1_TCDS')
env.reset()
action = np.asarray([0.5, 0.0])
print("action: {}".format(action))
while True:
    env.render()
    print("action: {}".format(action))
    env.step(action)
    
    time.sleep(1.0)
    

    