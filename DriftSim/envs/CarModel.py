from WheelModel import *
from TireModel import *
import numpy as np

class Car:

    DEBUG_NF = True
    DEBUG_KIN = True
    DEBUG_PT = True

    def __init__(self):
        # Modelled after Mercedes CLS 63 AMG
        # Constants
        self.wheelbase = 2.854
        self.track = 1.446
        self.cgz = 0.53
        self.mass = 2220 # kg
        self.Iz = 1549   # kg.m^2
        self.mu = 1

        # Powertrain
        self.eng_angvel = 0
        self.Ie = 0.44
        self.Iw = 1.85

        self.steer_ang = 0
        self.eng_cmd = 0

        # CG Kinematics
        self.ax = 0
        self.ay = 0
        self.vx = 0
        self.vy = 0

        self.yaw_rate = 0

        # World Kinematics
        self.xw = 0
        self.yw = 0
        self.yaw = 0 # car angle wrt world
        self.phi = 0 # motion heading

        # CG Dynamics
        self.fx_cg = 0
        self.fy_cg = 0
        self.mz_cg = 0

        # init all 4 tires here
        self.FL = FrontWheel(self.wheelbase/2, self.track/2)
        self.FR = FrontWheel(self.wheelbase/2, -self.track/2)
        self.RR = RearWheel(-self.wheelbase/2, -self.track/2)
        self.RL = RearWheel(-self.wheelbase/2, self.track/2)

        self.FL.normal = self.mass/4
        self.FR.normal = self.mass/4
        self.RR.normal = self.mass/4
        self.RL.normal = self.mass/4

        self.FL_ang_imp = 0
        self.FR_ang_imp = 0
        self.RR_ang_imp = 0
        self.RL_ang_imp = 0

        self.run = 0
        self.ctr = 0

    def resetSim(self, vel, px, py):
        self.__init__()

        self.xw = px
        self.yw = py
        self.vx = vel

        self.FL.ang_vel = 2*vel/FL.wheel_dia
        self.FR.ang_vel = 2*vel/FR.wheel_dia
        self.RR.ang_vel = 2*vel/RR.wheel_dia
        self.RL.ang_vel = 2*vel/RL.wheel_dia

    def getState(self):
        ctrl = (self.steer_ang, self.eng_cmd)
        w_kin = (self.xw, self.yw, self.yaw, self.phi)
        cg_kin = (self.ax, self.ay, self.vx, self.vy)
        cg_dyn = (self.fx_cg, self.fy_cg, self.mz_cg)
        return {"control" : ctrl , "world" : w_kin, "kin" : cg_kin, "dyn" : cg_dyn}

    def explicitControl(self, steer_ang, eng_cmd, dt):
        self.steer_ang = steer_ang
        self.eng_cmd = eng_cmd
        self.applyControl(0, 0, dt)

    def applyControl(self, dsteer_ang, deng_cmd, dt):
        self.steer_ang += dsteer_ang * dt
        self.eng_cmd += deng_cmd * dt

        self._update_steer_ang(self.steer_ang)
        self._apply_ground_imp(dt)
        self._apply_engine_imp(self.eng_cmd, dt)
        self._update_kinematics(dt/2)

        self._update_net_force()
        self._update_norms()

        self._solve_powertrain(dt)
        self._update_kinematics(dt/2)

    def _update_steer_ang(self, angle):
        self.steer_ang = angle
        if abs(angle) < 0.01:
            self.FL.steer_ang = 0.0
            self.FR.steer_ang = 0.0
            return

        R = self.wheelbase / np.tan(angle)
        self.FL.steer_ang = np.arctan2(self.wheelbase, (R - self.track/2))
        self.FR.steer_ang = np.arctan2(self.wheelbase, (R + self.track/2))

    def _update_kinematics(self, dt):
        # Car frame
        self.ax = self.fx_cg/self.mass
        self.ay = self.fy_cg/self.mass
        self.vx += self.ax * dt
        self.vy += self.ay * dt

        self.yaw_rate += dt*self.mz_cg/self.Iz

        # World frame
        self.yaw += self.yaw_rate*dt
        self.phi = np.arctan2(self.vy,self.vx) # heading
        self.xw += dt*(self.vx*np.cos(self.phi) - self.vy*np.sin(self.phi))
        self.yw += dt*(self.vx*np.sin(self.phi) + self.vy*np.cos(self.phi))

        if self.DEBUG_KIN:
            print("")
            print("_____________Kinematics Update________________")
            print("vx:", self.vx)
            print("vy:", self.vy)
            print("ax:", self.ax)
            print("ay:", self.ay)
            print("")
            print("xw:", self.xw)
            print("yw:", self.yw)
            print("yaw:", self.yaw)

    def _update_net_force(self):
        #compute net force on cg from each of the tires
        FL_x, FL_y = self.FL.computeForces(self.mu, self.vx, self.vy, self.yaw_rate)
        FR_x, FR_y = self.FR.computeForces(self.mu, self.vx, self.vy, self.yaw_rate)

        RR_x, RR_y = self.RR.computeForces(self.mu, self.vx, self.vy, self.yaw_rate)
        RL_x, RL_y = self.RL.computeForces(self.mu, self.vx, self.vy, self.yaw_rate)

        self.fx_cg = FL_x + FR_x + RR_x + RL_x
        self.fy_cg = FL_y + FR_y + RR_y + RL_y

        self.mz_cg = -FL_x * self.FL.pos[1] + FL_y * self.FL.pos[0] \
                     -FR_x * self.FR.pos[1] + FR_y * self.FR.pos[0] \
                     -RR_x * self.RR.pos[1] + RR_y * self.RR.pos[0] \
                     -RL_x * self.RL.pos[1] + RL_y * self.RL.pos[0]
        self.run += self.mz_cg
        self.ctr += 1

        if self.DEBUG_NF:
            print("")
            print("_____________Net Force Update________________")
            print("vx:", self.vx)
            print("vy:", self.vy)
            print("yaw_rate:", self.yaw_rate)
            print("")
            print("fx:", FL_x,FR_x,RR_x,RL_x)
            print("fy:", FL_y,FR_y,RR_y,RL_y)
            print("mz avg:", self.run/self.ctr)
            print("ang_vel:", self.FL.ang_vel, self.FR.ang_vel, self.RR.ang_vel, self.RL.ang_vel)

    def _update_norms(self):
        # X front, Y left
        self.FL.normal = self.mass/4 #- self.mass/2 * self.cgz * (self.ay / self.FL.pos[1] + self.ax / self.FL.pos[0])
        self.FR.normal = self.mass/4 #- self.mass/2 * self.cgz * (self.ay / self.FR.pos[1] + self.ax / self.FR.pos[0])
        self.RR.normal = self.mass/4 #- self.mass/2 * self.cgz * (self.ay / self.RR.pos[1] + self.ax / self.RR.pos[0])
        self.RL.normal = self.mass/4 #- self.mass/2 * self.cgz * (self.ay / self.RL.pos[1] + self.ax / self.RL.pos[0])

    def _apply_engine_imp(self, tau_in, dt):
        dL_in = tau_in * dt ##################
        dL_e = dL_in * self.Ie / (self.Ie + 4*self.Iw)

        self.eng_angvel += dL_e/self.Ie

        self.FL_ang_imp = (dL_in - dL_e)/(4*self.Iw)
        self.FR_ang_imp = (dL_in - dL_e)/(4*self.Iw)
        self.RR_ang_imp = (dL_in - dL_e)/(4*self.Iw)
        self.RL_ang_imp = (dL_in - dL_e)/(4*self.Iw)

        self.FL.ang_vel += self.FL_ang_imp
        self.FR.ang_vel += self.FR_ang_imp
        self.RR.ang_vel += self.RR_ang_imp
        self.RL.ang_vel += self.RL_ang_imp

        if self.DEBUG_PT:
            print("")
            print("_____________Input Update________________")
            print("dL_in:", dL_in)
            print("dL_e:", dL_e)


    def _apply_ground_imp(self, dt):
        self.FL.ang_vel -= self.FL.getBacktorque()*dt/self.Iw
        self.FR.ang_vel -= self.FR.getBacktorque()*dt/self.Iw
        self.RR.ang_vel -= self.RR.getBacktorque()*dt/self.Iw
        self.RL.ang_vel -= self.RL.getBacktorque()*dt/self.Iw

############# NEED TO FIX
    def _solve_powertrain(self, dt): ### FIX

        #dL_front = 2*dL_e*self.Iw/self.Ie + dt*(self.FL.getBacktorque() + self.FR.getBacktorque())
        #dL_rear = 2*dL_e*self.Iw/self.Ie + dt*(self.RL.getBacktorque() + self.RR.getBacktorque())

        dL_front = dt*(self.FL.getBacktorque() + self.FR.getBacktorque())
        dL_rear = dt*(self.RL.getBacktorque() + self.RR.getBacktorque())

        FL_del = (dL_front-self.FL.getBacktorque()*dt)/(self.Iw)
        FR_del = (dL_front-self.FR.getBacktorque()*dt)/(self.Iw)
        RR_del = (dL_rear-self.RR.getBacktorque()*dt)/(self.Iw)
        RL_del = (dL_rear-self.RL.getBacktorque()*dt)/(self.Iw)
##############

        self.FL.ang_vel += FL_del
        self.FR.ang_vel += FR_del
        self.RR.ang_vel += RR_del
        self.RL.ang_vel += RL_del

        if self.DEBUG_PT:
            print("")
            print("_____________Powertrain Update________________")
            print("dL_front:", dL_front)
            print("dL_rear:", dL_rear)
            print("")
            print("ang_vel deltas:", FL_del, FR_del, RR_del, RL_del)
