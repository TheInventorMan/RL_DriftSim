from WheelModel import *
from TireModel import *
import numpy as np

class Car:
    def __init__(self):
        # Constants
        self.wheelbase = 2.5
        self.track = 1.5
        self.cgz = 0.75
        self.mass = 1500 # kg
        self.Iz = 100 # ???
        self.mu = 1

        # Powertrain
        self.eng_angvel = 0
        self.Ie = 100
        self.Iw = 100

        self.steer_ang = 0

        # CG Kinematics
        self.ax = 0
        self.ay = 0
        self.vx = 0
        self.vy = 0

        self.yaw_rate = 0

        # World Kinematics
        self.xw = 0
        self.yw = 0
        self.yaw = 0
        self.phi = 0

        # CG Dynamics
        self.fx_cg = 0
        self.fy_cg = 0
        self.mz_cg = 0

        # init all 4 tires here
        self.FL = FrontLeftWheel(self.wheelbase/2, self.track/2)
        self.FR = FrontRightWheel(self.wheelbase/2, -self.track/2)
        self.RR = RearRightWheel(-self.wheelbase/2, -self.track/2)
        self.RL = RearLeftWheel(-self.wheelbase/2, self.track/2)

        self.FL.normal = self.mass/4
        self.FR.normal = self.mass/4
        self.RR.normal = self.mass/4
        self.RL.normal = self.mass/4

    def applyControl(self, steer_ang, eng_cmd, dt):
        self.update_steer_ang(steer_ang)
        self.update_kinematics(dt)
        self.update_net_force()
        self.update_norms()
        self.solve_powertrain(eng_cmd, dt)

    def update_steer_ang(self, angle):
        self.steer_ang = angle
        R = self.wheelbase / np.tan(angle)
        self.FL.steer_ang = np.arctan2(self.wheelbase, (R - self.track/2))
        self.FR.steer_ang = np.arctan2(self.wheelbase, (R + self.track/2))

    def update_kinematics(self, dt):
        # Car frame
        self.ax = self.fx_cg/self.mass
        self.ay = self.fy_cg/self.mass
        self.vx += self.ax * dt
        self.vy += self.ay * dt

        self.yaw_rate = dt*self.mz_cg/self.Iz

        # World frame
        self.yaw += self.yaw_rate*dt
        self.phi = self.yaw - np.arctan2(self.vy,self.vx)
        self.xw += dt*(self.vx*np.cos(self.yaw) + self.vy*np.sin(self.yaw))
        self.yw += dt*(-self.vx*np.sin(self.yaw) + self.vy*np.cos(self.yaw))

    def update_net_force(self):
        #compute net force on cg from each of the tires
        FL_x, FL_y = self.FL.computeForces(self.mu, self.vx, self.vy, self.yaw_rate)
        FR_x, FR_y = self.FR.computeForces(self.mu, self.vx, self.vy, self.yaw_rate)

        RR_x, RR_y = self.RR.computeForces(self.mu, self.vx, self.vy, self.yaw_rate)
        RL_x, RL_y = self.RL.computeForces(self.mu, self.vx, self.vy, self.yaw_rate)

        self.fx_cg = FL_x + FR_x + RR_x + RL_x
        self.fy_cg = FL_y + FR_y + RR_y + RL_y

        self.mz_cg = -FL_x * self.FL.pos[1] + FL_y * self.FL.pos[0] \
                     -FR_x * self.FL.pos[1] + FL_y * self.FL.pos[0] \
                     -RR_x * self.RR.pos[1] + RR_y * self.RR.pos[0] \
                     -RL_x * self.RL.pos[1] + RL_y * self.RL.pos[0]

    def update_norms(self):
        # X front, Y left
        self.FL.normal = self.mass/4 - self.mass/2 * self.cgz * (self.ay / self.FL.pos[1] + self.ax / self.FL.pos[0])
        self.FR.normal = self.mass/4 - self.mass/2 * self.cgz * (self.ay / self.FR.pos[1] + self.ax / self.FR.pos[0])
        self.RR.normal = self.mass/4 - self.mass/2 * self.cgz * (self.ay / self.RR.pos[1] + self.ax / self.RR.pos[0])
        self.RL.normal = self.mass/4 - self.mass/2 * self.cgz * (self.ay / self.RL.pos[1] + self.ax / self.RL.pos[0])

    def solve_powertrain(self, tau_in, dt):

        dL_in = tau_in * dt
        dL_e = dL_in * self.Ie / (self.Ie + 4*self.Iw)

        self.eng_angvel += dL_e/self.Ie

        dL_front = 2*dL_e*self.Iw/self.Ie + dt*(self.FL.getBacktorque() + self.FR.getBacktorque())
        dL_rear = 2*dL_e*self.Iw/self.Ie + dt*(self.RL.getBacktorque() + self.RR.getBacktorque())

        self.FL.ang_vel += (dL_front-self.FL.getBacktorque())/(2*self.Iw)
        self.FR.ang_vel += (dL_front-self.FR.getBacktorque())/(2*self.Iw)
        self.RR.ang_vel += (dL_rear-self.RR.getBacktorque())/(2*self.Iw)
        self.RL.ang_vel += (dL_rear-self.RL.getBacktorque())/(2*self.Iw)
