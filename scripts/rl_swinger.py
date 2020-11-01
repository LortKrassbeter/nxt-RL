# this program implements a simple learning algorithm on the hardware
import nxt
from nxt.motor import *
from nxt.sensor import *

import numpy as np

from tqdm import tqdm
from threading import Thread, Event
import sys, time, traceback

# setup hardware
b = nxt.locator.find_one_brick(debug=True)
light = Light(b, PORT_1)
motor = Motor(b, PORT_A)
motor.reset_position(True)
motor.reset_position(False)

# training setup
episodes = 20
episode_length = 50 # in actions

# lightness calibration
dt = 1e-2 # s between lightness measurements
ti = 3 # integration time
ls = []
for i in range(int(ti//dt)):
    time.sleep(dt)
    l = light.get_lightness()
    ls.append(l)
LNESS_THRES = np.mean(ls) + 3*np.std(ls) # lightness must be above 3sigma to count

print("lightness threshold = {}".format(LNESS_THRES))

# define the environment step
class SwingStep:
    def __init__(self, motor, light, readyevent):
        self.readyevent = readyevent
        self.motor = motor
        self.light = light
        self.lightness = []
        self.angles = []
        self.reward = 0
    
    def act(self, power, angle):
        if angle == 0:
            time.sleep(1)
        else:
            self.motor.turn(power, angle)
        self.readyevent.set()
    
    def observe(self, dt):
        t = 0
        while not self.readyevent.is_set():
            lness = self.light.get_lightness()
            self.lightness.append(lness)
            if lness > LNESS_THRES:
                self.reward += 1
            self.angles.append(self.motor.get_tacho().tacho_count % 360)
            t += dt

def reset_motor_pos(motor, reset_angle=155, power=60, wait=1):
    #motor.reset_position(True)
    #motor.reset_position(False)

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

# observation containers
rewards_t = []
lness_t = []
angles_t = [[motor.get_tacho().tacho_count % 360]]

# action space
rotspace = np.array([0, 90, 180, 360])
powerspace = np.array([100, -100])
actiondim = len(rotspace)*len(powerspace)

# state space
dangle = 30
anglespace = np.arange(0, 360, dangle)
statedim = anglespace.shape[0]

# define Q-values
Q = np.zeros((statedim, actiondim))#np.random.random(size=(len(rotspace), len(powerspace))

epsilon = 0.8
decay = 0.9
gamma = 0.9
min_epsilon = 0
alpha = 0.9

init_angle = 155

timestamp = time.time()

for ei in range(episodes):
    print("Starting episode {}...".format(ei+1))
    print("epsilon = {}".format(epsilon))
    # reset to initial position
    time.sleep(2)
    reset_motor_pos(motor, reset_angle=init_angle, wait=2)

    for ai in tqdm(range(episode_length)):
        
        # perform epsilon-greedy policy
        curangle = angles_t[-1][-1]
        beforestate = np.digitize(curangle, anglespace, right=True)
       
        #print("action {} in state {}°".format(ai + 1, curangle))

        if epsilon > np.random.random():
            actionid = np.random.randint(actiondim)
            irot = actionid // len(powerspace)
            ipow = actionid % len(powerspace)
        else:
            actionid = np.random.choice(np.flatnonzero(Q[beforestate - 1, :] == Q[beforestate - 1, :].max()))
            irot = actionid // len(powerspace)
            ipow = actionid % len(powerspace)
        
        rot = rotspace[irot]
        power = powerspace[ipow]
        
        #print("action: rotation by {}° with {} power".format(rot, power))
        
        # execute action & observe environment, reward
        ready = Event()
        step = SwingStep(motor, light, ready)
        tm = Thread(target=step.act, args=(power, rot))
        tl = Thread(target=step.observe, args=(dt,))
        tm.start()
        tl.start()
        tm.join()
        tl.join()
        
        rewards_t.append(step.reward)
        lness_t.append(step.lightness)
        angles_t.append(step.angles)
        
        # new state
        curangle = angles_t[-1][-1]
        afterstate = np.digitize(curangle, anglespace, right=True)

        # update Q-values
        Q[beforestate - 1, actionid] = (1 - alpha)*Q[beforestate - 1, actionid] + alpha*(step.reward + gamma*np.max(Q[afterstate - 1, actionid]))
    
    # decay epsilon
    epsilon = max(min_epsilon, decay*epsilon)
    print("avg reward = {}".format(np.mean([np.mean(li) for li in rewards_t[-episode_length:]])))
    
# save data
np.save("rewards.npy", np.array(rewards_t))
np.save("q_values.npy", Q)

light.set_illuminated(False)
motor.idle()
