import gym
from gym import error, spaces, utils
from TireModel import *
from WheelModel import *
from CarModel import Car
from WorldModel import World

class DriftSim(gym.Env):

    world_dims = (200, 25) # meters
    img_dims = (25, 25) # meters
    goal_coords = (200, 12.5) # meters
    start_coords = (0, 12.5)

    obst_list = [((0,0),(0,1),(200,1),(200,0)),
                 ((0,25),(0,24),(200,24),(200,25))]

    resolution = 100 # px per meter
    start_vel = 40 #m/s
    car_dims = (0,0)

    def __init__(self):
        super(DriftSim, self).__init__()
        self.reset()

    def step(self, action):
        # Execute one time step within the environment
        pass

    def reset(self):
        # Reset the state of the environment to an initial state
        self.ego = Car()
        self.ego.resetSim(self.start_vel, self.start_coords[0], self.start_coords[1])
        self.car_dims = (self.ego.wheelbase, self.ego.track)
        self.world = World(self.world_dims, self.img_dims, self.resolution, self.goal_coords, self.car_dims, self.obst_list)

    def render(self, mode, close=False):
        # Render the environment to the screen
        pass
