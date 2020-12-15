from pynput.mouse import Button, Controller
from pynput import keyboard
import time


mouse = Controller()

while True:
    print('Mouse Position: {}'.format(mouse.position))
    # with keyboard.Events() as events:
    #     for event in events:
    #         print("Event: {}".format(event))
    with keyboard.Events() as events:
        event = events.get(0.01)
        if event is None:
            print("timeout")
        else:
            print("received event {}".format(event))
    
    time.sleep(0.5)
