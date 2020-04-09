from TireModel import Tire
import numpy as np

class FrontWheel(Tire):
    def __init__(self, dx, dy):
        super().__init__()
        self.wheel_dia = 0.6475
        self.pos = (dx, dy)
        self.steer_ang = 0

        self.normal = 0
        self.ang_vel = 0
        self.I = 1.85
        self.L = 0

        self.fx = 0
        self.fy = 0

    def getBacktorque(self):
        return self.fx * self.wheel_dia/2

    def computeForces(self, mu, car_velx, car_vely, yaw_rate):
        tire_translation = self.ang_vel * self.wheel_dia/2
        car_velx -= yaw_rate*self.pos[1]
        car_vely += yaw_rate*self.pos[0]
        self.L = self.I * self.ang_vel

        vx = car_velx * np.cos(self.steer_ang) - car_vely * np.sin(self.steer_ang)
        vy = car_velx * np.sin(self.steer_ang) + car_vely * np.cos(self.steer_ang)

        tire_fx, tire_fy = self.getContactForces(self.normal, mu, tire_translation, vx, vy)

        self.fx = tire_fx * np.cos(self.steer_ang) + tire_fy * np.sin(self.steer_ang)
        self.fy = tire_fx * -np.sin(self.steer_ang) + tire_fy * np.cos(self.steer_ang)

        return self.fx, self.fy

class RearWheel(Tire):
    def __init__(self, dx, dy):
        super().__init__()
        self.pos = (dx, dy)
        self.wheel_dia = 0.6475

        self.normal = 0
        self.ang_vel = 0
        self.I = 1.85
        self.L = 0

        self.fx = 0
        self.fy = 0

    def getBacktorque(self):
        return self.fx * self.wheel_dia/2

    def computeForces(self, mu, car_velx, car_vely, yaw_rate):
        car_velx -= yaw_rate*self.pos[1]
        car_vely += yaw_rate*self.pos[0]
        tire_translation = self.ang_vel * self.wheel_dia/2
        self.L = self.I * self.ang_vel

        self.fx, self.fy = self.getContactForces(self.normal, mu, tire_translation, car_velx, car_vely)

        return self.fx, self.fy
