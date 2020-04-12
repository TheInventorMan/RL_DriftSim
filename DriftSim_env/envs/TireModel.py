import numpy as np

class Tire:

    ## Coordinate system:
    # X towards the front
    # Y to the left

    ### Pacejka coefficients ###
    # Modelled after Hankook 205/65R15 6.0J K406
    # Lateral coeffs
    a = [1.55, -55, 1350, 1400,
         7.2, 0.014, -0.24, 1.0,
        -0.09, 0, 0, -0.9, 0, 0, 0]

    # Longitudinal coeffs
    b = [1.6, -75, 1600, 23.3, 410,
         0.075, 0, 0.055, -0.024, 0, 0]

    # Aligning coeffs
    c = [2.2, -4.3, -4.4, -1.9,
        -9.6, 0.0225, 0, 0.044,
        -0.58, 0.18, 0.043,
         0.048, -0.0035, -0.18,
         0.14, -1.029, 0.27, -1.1]

    # Combined coeffs
    g = [24.8565, 22.43545,
         11.14388, 10.14162]

    def __init__(self):

        ### Internal Pacejka States ###
        self.slip_r = 0
        self.slip_ang = 0
        self.peakf_slip_r = 0
        self.peakf_slip_ang = 0
        self.sh = [0] * 20
        self.ah = [0] * 20
        self.sigma_hat = 0
        self.alpha_hat = 0

        # Intermediate forces
        self.fx0 = 0
        self.fy0 = 0
        self.gx = 0
        self.gy = 0

        # Output forces
        self.fx = 0
        self.fy = 0
        self.mz = 0

        self.max_fx = 0
        self.max_fy = 0
        self.max_mz = 0

        # Kinematics
        self.vx = 0
        self.vy = 0
        self.ang_vel = 0

        # Initialize peak force values
        self._init_peakf()

    def getContactForces(self, N, mu, rot_trans, vx, vy):

        self.vx = vx
        self.vy = vy
        self.fz = min(N*0.001, 30)

        ## Find sigma, alpha, sh, ah
        sigma = (rot_trans - vx) / max(abs(vx), 0.001)
        alpha = -np.arctan2(vy / max(abs(vx), 0.001), 1) * 180 / np.pi

        self.fx0 = self._compute_Fx(sigma, self.fz, mu)
        self.fy0 = self._compute_Fy(alpha, self.fz, mu)
        self.mz = self._compute_Mz(alpha, self.fz, mu)
        self.gx = self._compute_Gx(sigma, alpha)
        self.gy = self._compute_Gy(sigma, alpha)

        self.fx = self.fx0 * self.gx
        self.fy = self.fy0 * self.gy

        self.slip_r = sigma
        self.slip_ang = alpha * np.pi/180

        self._compute_peakf(N)
        self.peakf_slip_r = self.sigma_hat
        self.peakf_slip_ang = self.alpha_hat * np.pi/180

        return (self.fx, self.fy)

    def _init_peakf(self):
        for i in range(20):
            N = (i+1)/2
            f_max = 0
            x_max = 1
            dx = [i/100 for i in range(100)]

            for x in dx:
                Fx = self._compute_Fx(x, N, 1)
                if (Fx > f_max):
                    self.sh[i] = x
                    f_max = Fx
                elif (Fx < f_max and f_max > 0):
                    break

            f_max = 0
            y_max = 40
            dy = [i*40/100 for i in range(100)]
            for y in dy:
                Fy = self._compute_Fy(y, N, 1)
                if (Fy > f_max):
                    self.ah[i] = y
                    f_max = Fy
                elif (Fy < f_max and f_max > 0):
                    break
        return

    def _compute_peakf(self, N):
        n = max(0, min(N/500.0-1, 18.999))
        i = int(n)
        avg_factor = n - i
        self.sigma_hat = self.sh[i] * (1-avg_factor) + self.sh[i+1] * avg_factor
        self.alpha_hat = self.ah[i] * (1-avg_factor) + self.ah[i+1] * avg_factor


    def _compute_Fx(self, sigma, Fz, mu):

        b = self.b

        C = b[0]
        D = (b[1] * Fz + b[2]) * Fz
        BCD = (b[3] * Fz + b[4]) * Fz * np.exp(-b[5] * Fz)
        B =  BCD / (C * D)
        E = b[6] * Fz**2 + b[7] * Fz + b[8] # curvature factor
        Sh = b[9] * Fz + b[10]
        S = 100 * sigma + Sh

        self.max_fx = D * mu
        fx = mu * D * np.sin(C * np.arctan2(B * S - E * (B * S - np.arctan2(B * S, 1)), 1))
        return fx


    def _compute_Fy(self, alpha, Fz, mu):

        a = self.a

        C = a[0] #shape factor
        D = (a[1] * Fz + a[2]) * Fz #peak factor
        BCD = a[3] * np.sin(2 * np.arctan2(Fz / a[4], 1))
        B = BCD / (C * D) #stiffness factor
        E = a[6] * Fz + a[7] #curvature factor
        Sh = a[9] * Fz + a[10] #horizontal shift
        Sv = a[13] * Fz + a[14] #vertical shift
        S = alpha + Sh #composite slip angle

        #lateral force
        self.max_fy = (D + Sv) * mu
        fy = mu * D * np.sin(C * np.arctan2(B * S - E * (B * S - np.arctan2(B * S, 1)), 1)) + Sv
        return fy

    def _compute_Mz(self, alpha, Fz, mu):
        c = self.c

        C = c[0]
        D = (c[1] * Fz + c[2]) * Fz                            #peak factor
        BCD = (c[3] * Fz + c[4]) * Fz * np.exp(-c[5] * Fz)
        B =  BCD / (C * D)                                     #stiffness factor
        E = c[7] * Fz * Fz + c[8] * Fz + c[9]                  #curvature factor
        Sh = c[12] * Fz + c[13]                                #horizontal shift
        S = alpha + Sh                                         #composite slip angle
        Sv = c[16] * Fz + c[17]                                #vertical shift

        #self-aligning torque
        self.max_mz = (D + Sv) * mu
        mz = mu * D * np.sin(c[0] * np.arctan(B * S - E * (B * S - np.arctan(B * S)))) + Sv
        return mz

    def _compute_Gx(self, sigma, alpha):
        g = self.g
        alpha *= np.pi/180
        B = g[0] / np.sqrt(1 + (g[1]*sigma)**2)
        gx = 1 / np.sqrt(1 + (B*alpha)**2)
        return gx

    def _compute_Gy(self, sigma, alpha):
        g = self.g
        alpha *= np.pi/180
        B = g[2] / np.sqrt(1 + (g[3]*alpha)**2)
        gy = -1 / np.sqrt(1 + (B*sigma)**2)
        return gy
