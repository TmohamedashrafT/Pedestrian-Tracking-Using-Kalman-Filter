import numpy as np
class Kalman_filter:
    def __init__(self):
        self.missing = 1
        self.track_id = None
        num_of_pred, dt, noise = 8, 1,10
        self.X = np.zeros(num_of_pred)
        # In the initial stage we assume that there is no Convariance between the outputs
        self.P = np.diag(noise * np.ones(num_of_pred))
        self.P[4:,4:] *= 1000
        ##initialize the state-transition matrix A
        ## (x_min, y_min, x_max, y_max, x_min_velocity, y_min_velocity, x_max_velocity, y_max_velocity)
        self.A = np.eye(num_of_pred)
        for i in range(num_of_pred // 2 ):
            self.A[i, num_of_pred // 2 + i] = dt

        ## initialize the process covariance
        self.Q = np.diag(0.5 * np.ones(num_of_pred))
        ## Because the detection model will only provide the position, we initialize the Measurement matrix as (4, 8)
        self.H = np.eye(num_of_pred // 2, num_of_pred)

        self.R = np.diag(noise * np.ones(num_of_pred // 2))
    def predict(self):
        # assuming constant velocity model, so there is no acceleration(u)
        # X = AX + Bu
        self.X = np.dot(self.A, self.X)
        # P = APA.T + Q
        self.P = np.linalg.multi_dot((self.A, self.P, self.A.T)) + self.Q
    def update(self,measurements):
        # Calc Kalman Gain k = PH.T(HPH.T + R)
        S      = np.linalg.multi_dot((self.H, self.P, self.H.T)) + self.R
        K_gain = np.linalg.multi_dot((self.P,self.H.T, np.linalg.inv(S)))
        # correction
        y = measurements - np.dot(self.H, self.X)
        self.X = self.X + np.dot(K_gain, y)
        self.P = self.P - np.linalg.multi_dot((K_gain, self.H, self.P))
