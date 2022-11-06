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
                            [1, 0, 0],
                            [0, 1, 0]
                        ]
                        )

        self.C_trans = np.transpose(self.C)
        # State transition error covariance
        self.R = np.array([[1.0,0,0],[0,1.0,0],[0,0,1.0]])
        # Measurement error
        self.Q = np.array([[0.75,0],[0,0.75]])


    def predict(self, u):
        # with open("/root/catkin_ws/src/predict.txt", "a") as f:
        #             np.savetxt(f, self.R)
        #             f.write("----------------------------------------------\n")
        with open("/root/catkin_ws/src/covariance_no_gps.txt", "a") as f:
                    np.savetxt(f, [self.P[0][0]])
                    # f.write("----------------------------------------------\n")
        # self.x= self.A @ self.x + self.B @ u
        self.x = np.matmul(self.A,self.x) + np.matmul(self.B,u)
        # self.P = self.A @ self.P @ np.transpose(self.A) + self.R
        self.P = np.matmul(np.matmul(self.A,self.P),np.transpose(self.A)) + self.R



        
        # raise NotImplementedError

    def update(self, z):
        
        # with open("/root/catkin_ws/src/covariance.txt", "a") as f:
        #     np.savetxt(f, self.Q)
        #     f.write("----------------------------------------------\n")

        inverse_we_want = np.linalg.inv(np.matmul(self.C,np.matmul(self.P,self.C_trans)) + self.Q)

        kalman_gain = np.matmul(self.P,np.matmul(self.C_trans,inverse_we_want));

        self.x += np.matmul(kalman_gain,z - np.matmul(self.C,self.x))
        
        self.P = np.matmul(np.eye(3) - np.matmul(kalman_gain,self.C),self.P)
        
        with open("/root/catkin_ws/src/covariance3000000.txt", "a") as f:
                    f.write("----------------------------------------------\n")
                    np.savetxt(f, [self.P[0][0]])
                    f.write("----------------------------------------------\n")
        # raise NotImplementedError
        return self.x, self.P
