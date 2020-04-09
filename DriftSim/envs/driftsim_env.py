import gym
from gym import error, spaces, utils
from TireModel import *
from WheelModel import *
from CarModel import Car
from WorldModel import World

class DriftSim(gym.Env):
    metadata = {'render.modes': ['human']}

    MIN_STEER_ANG = -25 # degrees
    MAX_STEER_ANG = 25
    MIN_ENG_CMD = -1000 # N.m
    MAX_ENG_CMD = 250

    world_dims = (200, 25) # meters
    img_dims = (25, 25) # meters
    goal_coords = (200, 12.5) # meters
    start_coords = (0, 12.5)

    obst_list = [((0,0),(0,1),(200,1),(200,0)),
                 ((0,25),(0,24),(200,24),(200,25))]

    resolution = 100 # px per meter
    start_vel = 40 #m/s
    car_dims = (0,0)
    dt = 0.001

    def __init__(self):
        super(DriftSim, self).__init__()

        self.action_space = spaces.Box(low=np.array([self.MIN_STEER_ANG, self.MIN_ENG_CMD]),
                                       high=np.array([self.MAX_STEER_ANG, self.MAX_ENG_CMD]),
                                       dtype=np.float32)

        obs_shape = (self.img_dims[0]*self.resolution, self.img_dims[1]*self.resolution)
        lower_bound = np.dstack((np.full(obs_shape, 0.0), np.full(obs_shape, -1000.0)))
        upper_bound = np.dstack((np.full(obs_shape, 1.0), np.full(obs_shape, 1000.0)))
        self.observation_space = spaces.Box(low=lower_bound, high=upper_bound, dtype=np.float32)

        self.reset()

    def step(self, action):
        # Execute one time step within the environment
        self.ego.applyControl(action[0], action[1], self.dt)
        self.state = self.ego.getState()

        xw = self.state["world"][0]
        yw = self.state["world"][1]
        theta = self.state["world"][2] + self.state["world"][3]

        img, pot_val, collision, complete = self.world.observe_world(xw, yw, theta)

        meta_layer = np.zeros(img.shape)
        meta_layer[0][0:2] = self.state["ctrl"]
        meta_layer[1][0:4] = self.state["world"]
        meta_layer[2][0:4] = self.state["cg"]

        obs = np.dstack((img, meta_layer))
        reward = pot_val - self.potfield_value
        self.potfield_value = pot_val
        done = collision or complete

        return obs, reward, done, {}

    def reset(self):
        # Reset the state of the environment to an initial state
        self.ego = Car()
        self.ego.resetSim(self.start_vel, self.start_coords[0], self.start_coords[1])
        self.state = self.ego.getState()

        self.car_dims = (self.ego.wheelbase, self.ego.track)
        self.world = World(self.world_dims, self.img_dims, self.resolution, self.goal_coords, self.car_dims, self.obst_list)
        self.potfield_value = 0

    def render(self, mode='human', close=False):
        # Render the environment to the screen
        pass
