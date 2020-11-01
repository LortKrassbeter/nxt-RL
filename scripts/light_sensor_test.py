import nxt.locator
import nxt.brick
from nxt.sensor import *
import numpy as np
import sys, traceback, time

if '--help' in sys.argv:
    print("""Tests the light sensor at port 1

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
    #name, host, signal_strength, user_flash = b.get_device_info()
    s = Light(b, PORT_1)
    ls = []
    for i in range(100):
        time.sleep(0.05)
        l = s.get_lightness()
        ls.append(l)
        print(l)
    print("mean = ",np.mean(ls))
    print("std = ", np.std(ls))
    s.set_illuminated(False)
except:
    print("Error while running test:")
    traceback.print_tb(sys.exc_info()[2])
    print(str(sys.exc_info()[1]))
    if b in locals():
        b.sock.close()
