import nxt.locator
import nxt.brick
from nxt.motor import *
import numpy as np
import sys, traceback

if '--help' in sys.argv:
    print("""Tests a motor on port A

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
    m = Motor(b, PORT_A)
    ts = np.linspace(0, 2*np.pi, 10)
    power = 127 - 50*(np.sin(ts) + 1)*0.5
    angle = 60
    for i in range(len(power)):
        print("power: ",power[i])
        m.turn(int(power[i]), angle)
        m.turn(-int(power[i]), angle)
    m.idle()
except:
    print("Error while running test:")
    traceback.print_tb(sys.exc_info()[2])
    print(str(sys.exc_info()[1]))
    if b in locals():
        b.sock.close()

