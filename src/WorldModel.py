import numpy as np
from skimage.draw import polygon
import scipy

class WorldModel:

    def __init__(self, world_x, world_y, img_x, img_y, res, obst_list):
        self.img_x_px = int(img_x/res)
        self.img_y_px = int(img_y/res)
        self.world_x_px = int(world_x/res)
        self.world_y_px = int(world_y/res)

        self.world = np.zeros((self.world_x_px, self.world_y_px, 1), 'uint_8')

        for obst in obst_list:
            rr, cc = polygon(obst[:,0], obst[:,1], self.world.shape)
            self.world[rr,cc,0] = 1.0

    def get_image(self, x, y, theta): # fix zero indexing, left alignment
        # shift to center
        shifted = np.zeros(self.world.shape, 'uint_8')
        scipy.ndimage.shift(self.world, (-x, self.world_y_px//2 - y), output=shifted)

        # rotate about center
        rotated = np.zeros(shifted.shape, 'uint_8')
        scipy.ndimage.rotate(shifted, -theta, reshape=False, output=rotated)

        # shift to bottom
        scipy.ndimage.shift(rotated, (0, -self.world_y_px//2), output=shifted)

        # crop
        self.image = shifted[self.world_x_px//2 - self.img_x_px//2 : self.world_x_px//2 + self.img_y_px//2 , 0 : self.img_y_px, :]

        return self.image

    def get_reward(self, x, y, theta, track):
        # check for collision
        pass
