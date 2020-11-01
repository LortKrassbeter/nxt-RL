import nxt.locator
import nxt.brick
from nxt.motor import *
import numpy as np
import sys, traceback, time
from threading import Thread
if '--help' in sys.argv:
    print("""Tests the tacho/angle sensor

Usage: nxt_test           # Finds one brick and shows information about it
       nxt_test --verbose # Shows more debug information when trying to find the brick
       nxt_test --help    # Shows this help
""")
    exit(0)

debug = False
if '--verbose' in sys.argv or '--debug' in sys.argv:
    debug = True
    print('debug = True')

b = None
try:
    print('Find brick...', flush=True)
    b = nxt.locator.find_one_brick(debug=debug)

    motor = Motor(b, PORT_A)

    motor.reset_position(True)
    motor.reset_position(False)
    def turnmotor(motor, T):
        try:
            print("turnmotor started")
            t = 0 
            while t < T:
                t1 = time.time()
                motor.turn(100, 180)
                t += time.time() - t1
        except Exception as e:
            print(traceback.format_exception(*sys.exc_info()))
            raise
     
    def measure_angles(motor, dt, T, save=True):
        try:
            print("measure angle started")
            t = 0
            tacho_count = []
            block_tacho_count = []
            rotation_count = []
            while t < T:
                tacho = motor.get_tacho()
                tacho_count.append(tacho.tacho_count % 360)
                block_tacho_count.append(tacho.block_tacho_count % 360)
                rotation_count.append(tacho.rotation_count % 360)
                t += dt
                time.sleep(dt)
            if save:
                np.save("tacho_signal.npy", np.array(tacho_count))
                np.save("block_tacho_signal.npy", np.array(block_tacho_count))
                np.save("rotation_signal.npy", np.array(rotation_count))
        except Exception as e:
            print(traceback.format_exception(*sys.exc_info()))
            raise
     
    dt = 1/100
    T = 10

    #tm = Thread(target=turnmotor, args=(motor, T))
    tl = Thread(target=measure_angles, args=(motor, dt, T))

    #tm.start()
    tl.start()

    #tm.join()
    tl.join()

    motor.idle()

except:
    print("Error while running test:")
    traceback.print_tb(sys.exc_info()[2])
    print(str(sys.exc_info()[1]))
    if b in locals():
        b.sock.close()
