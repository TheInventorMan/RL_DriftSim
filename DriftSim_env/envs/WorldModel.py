import numpy as np
from skimage.draw import polygon
import scipy

class World:

    def __init__(self, world_dims, img_dims, res, goal_coords, car_dims, obst_list):
        self.img_x_px = int(img_dims[0]/res)
        self.img_y_px = int(img_dims[1]/res)
        self.world_x_px = int(world_dims[0]/res)
        self.world_y_px = int(world_dims[1]/res)
        self.res = res

        self.car_dims = car_dims
        self.goal_coords = goal_coords

        self.collision = False

        self.world = np.zeros((self.world_x_px, self.world_y_px, 1), 'uint_8')
        for obst in obst_list:
            obst_px = obst*self.res
            rr, cc = polygon(obst_px[:,0], obst_px[:,1], self.world.shape)
            self.world[rr,cc,0] = 1

    def observe_world(self, x, y, theta):
        # shift to center
        shifted = np.zeros(self.world.shape, 'uint_8')
        scipy.ndimage.shift(self.world, (self.world_x_px//2 - x, self.world_y_px//2 - y), output=shifted)

        # rotate about center
        rotated = np.zeros(shifted.shape, 'uint_8')
        scipy.ndimage.rotate(shifted, -theta, reshape=False, output=rotated)

        # crop
        self.image = rotated[self.world_x_px//2 - self.img_x_px//2 : self.world_x_px//2 + self.img_y_px//2,
                             self.world_y_px//2 - self.img_y_px//2 : self.world_y_px//2 + self.img_y_px//2, :]

        self.current_value = self._get_value(x, y, theta, wb, track)

        return self.image, self.current_value, self.collision, self.complete

    def _get_value(self, x, y, theta):
        # potential field value
        a_dist = 1000
        a_obst = 1000

        dist_val = a_dist - np.sqrt((self.goal_coords[0] - x)**2 + (self.goal_coords[1] - y)**2))

        wb, track = self.car_dims

        vehicle_val = sum(self.image[self.img_x_px//2 - self.res*track//2 : self.img_x_px//2 + self.res*track//2,
                          self.img_y_px//2 - self.res*wb//2 : self.img_y_px//2 + self.res*wb//2, :])

        self.collision = (vehicle_val > 0)
        self.complete = (abs(dist_val - a_dist) < 0.1)

        x_radius = self.res*(track//2 + 1.5)
        y_radius = self.res*(wb//2 + 2.5)
        obst_val = a_obst - sum(self.image[self.img_x_px//2 - x_radius : self.img_x_px//2 + x_radius,
                                self.img_y_px//2 - y_radius : self.img_y_px//2 + y_radius, :])

        value = dist_val + obst_val
        return value
