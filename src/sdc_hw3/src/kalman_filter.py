import numpy as np

class KalmanFilter():
    def __init__(self, x=0, y=0, yaw=0):
        # State [x, y, yaw]  3*1
        self.x = np.array([x, y, yaw])
        # Transition matrix
        self.A = np.identity(3)
        self.B = np.identity(3)
        # Error matrix == sigma_t
        self.P = np.identity(3) * 1
        # Observation matrix 
        self.C = np.array(
                        [
                            [1, 0],
                            [0, 1]
                        ]
                        )
        # State transition error covariance
        self.R = np.array([[0.05,0,0],[0,0.05,0],[0,0,0.05]])
        # Measurement error
        self.Q = np.array([[0.75,0],[0,0.75]])


    def predict(self, u):
        self.x = np.matmul(self.A,self.x) + np.matmul(self.B,u)
        self.P = np.matmul(np.matmul(self.A,self.P),np.transpose(self.A)) + self.R
        # raise NotImplementedError

    def update(self, z):

        kalman_gain = np.matmul(self.P[0:2,0:2],np.matmul(self.C,np.linalg.inv(np.matmul(np.matmul(self.C,self.P[0:2,0:2]),np.transpose(self.C)) + self.Q))
)
        self.x[0:2] += np.matmul(kalman_gain,z - np.matmul(self.C,self.x[0:2]))
        
        self.P[0:2,0:2] = np.matmul(np.eye(2) - np.matmul(kalman_gain,self.C),self.P[0:2,0:2])

        # raise NotImplementedError
        return self.x, self.P
