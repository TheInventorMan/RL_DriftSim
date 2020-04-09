import gym
from gym import error, spaces, utils
from TireModel import *
from WheelModel import *
from CarModel import Car
from WorldModel import World

class DriftSim(gym.Env):

    world_dims = (50, 200) # meters
    img_dims = (25, 25) # meters
    resolution = 100 # px per meter
    goal_coords = (25, 200) # meters
    start_coords = (25, 0)
    start_vel = 40 #m/s

    def __init__(self):
        super(DriftSim, self).__init__()


    def step(self, action):
        # Execute one time step within the environment
        pass

    def reset(self):
        # Reset the state of the environment to an initial state
        self.ego = Car()
        self.ego.resetSim(self.start_vel, self.start_coords[0], self.start_coords[1])
        self.world = World()
        pass

    def render(self, mode, close=False):
        # Render the environment to the screen
        pass
