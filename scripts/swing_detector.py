import nxt
import nxt.locator
from nxt.sensor import *
from nxt.motor import *
#import _thread
from threading import Thread
import sys, time, traceback
import numpy as np

b = nxt.locator.find_one_brick(debug=True)

light = Light(b, PORT_1)
motor = Motor(b, PORT_A)

motor.reset_position(True)

def turnmotor(motor, T):
    try:
        print("turnmotor started")
        t = 0 
        while t < T:
            t1 = time.time()
            motor.turn(100, 180)
            motor.turn(-100, 180)
            t += time.time() - t1
    except Exception as e:
        print(traceback.format_exception(*sys.exc_info()))
        raise
 
def measure_light(light, dt, T, save=True):
    try:
        print("measure light started")
        t = 0
        ls = []
        while t < T:
            l = light.get_lightness()
            ls.append(l)
            t += dt
            time.sleep(dt)

 
        if save:
            np.save("light_signal.npy", np.array(ls))
    except Exception as e:
        print(traceback.format_exception(*sys.exc_info()))
        raise
 
dt = 1/100
T = 10

tm = Thread(target=turnmotor, args=(motor, T))
tl = Thread(target=measure_light, args=(light, dt, T))

tm.start()
tl.start()

tm.join()
tl.join()

light.set_illuminated(False)
motor.idle()
