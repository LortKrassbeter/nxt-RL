import nxt
import nxt.locator
from nxt.motor import *
import time

b = nxt.locator.find_one_brick(debug=True)
motor = Motor(b, PORT_A)

def reset_motor_pos(motor, reset_angle=155, power=60, wait=1):
    motor.reset_position(True)
    motor.reset_position(False)

    cur_angle = motor.get_tacho().tacho_count % 360

    turn_angle = reset_angle % 360 - cur_angle
    if turn_angle > 0:
        s = 1
    else:
        s = -1
    s_power = int(s)*power
    abs_turn = abs(turn_angle)
    
    motor.turn(s_power, abs_turn)

    time.sleep(wait)
    
    motor.idle()

reset_motor_pos(motor)
