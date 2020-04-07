import numpy as np
from skimage.draw import polygon
import scipy

class World:

    def __init__(self, world_x, world_y, img_x, img_y, res, goal_coords, obst_list):
        self.img_x_px = int(img_x/res)
        self.img_y_px = int(img_y/res)
        self.world_x_px = int(world_x/res)
        self.world_y_px = int(world_y/res)

        self.goal_coords = goal_coords

        self.world = np.zeros((self.world_x_px, self.world_y_px, 1), 'uint_8')

        for obst in obst_list:
            rr, cc = polygon(obst[:,0], obst[:,1], self.world.shape)
            self.world[rr,cc,0] = 1

    def observe_world(self, x, y, theta, wb, track):
        # shift to center
        shifted = np.zeros(self.world.shape, 'uint_8')
        scipy.ndimage.shift(self.world, (self.world_x_px//2 - x, self.world_y_px//2 - y), output=shifted)

        # rotate about center
        rotated = np.zeros(shifted.shape, 'uint_8')
        scipy.ndimage.rotate(shifted, -theta, reshape=False, output=rotated)

        # crop
        self.image = rotated[self.world_x_px//2 - self.img_x_px//2 : self.world_x_px//2 + self.img_y_px//2,
                             self.world_y_px//2 - self.img_y_px//2 : self.world_y_px//2 + self.img_y_px//2, :]

        self.current_value = self.get_value(x, y, theta, wb, track)

        return self.image, self.current_value

    def get_value(self, x, y, theta, wb, track):
        # check for collision; potential field value
        dist_val = 1/(0.001+np.sqrt((self.goal_coords[0] - x)**2 + (self.goal_coords[1] - y)**2))
        return value
