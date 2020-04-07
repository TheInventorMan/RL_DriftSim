import gym
from gym import error, spaces, utils
from TireModel import *
from WheelModel import *
from CarModel import Car
from WorldModel import World

class DriftSim(gym.Env):
    def __init__(self):
        super(DriftSim, self).__init__()
        pass

    def step(self, action):
        # Execute one time step within the environment
        pass

    def reset(self):
        # Reset the state of the environment to an initial state
        pass

    def render(self, mode, close=False):
        # Render the environment to the screen
        pass
