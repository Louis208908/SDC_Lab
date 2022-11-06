#!/usr/bin/env python3

from tkinter import Scale
import rospy
import math, os
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped as Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from matplotlib import pyplot as plt
# KF coded by yourself
from kalman_filter import KalmanFilter

class Fusion:
    def __init__(self):
        rospy.Subscriber('/gps', Pose, self.gpsCallback)
        rospy.Subscriber('/radar_odometry', Odometry, self.odometryCallback)
        rospy.Subscriber('/gt_odom', Odometry, self.gtCallback)
        rospy.on_shutdown(self.shutdown)
        self.posePub = rospy.Publisher('/pred', Odometry, queue_size = 10)
        self.step = 0   # Record update times

        self.last_odometry_position = np.zeros(2)
        self.last_odometry_angle = 0

        self.gt_list = []
        self.est_list = []

    def shutdown(self):
        print("shuting down fusion.py")

    def gpsCallback(self, data):
        
        self.step += 1
        # Get GPS data only for 2D (x, y)
        measurement = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        gps_covariance = np.array(data.pose.covariance).reshape(6, 6)[:2,:2]    

        # KF update
        if self.step == 1:
            #raise NotImplementedError
            self.init_KF(measurement[0], measurement[1], 0)
        else:
            #raise NotImplementedError
            #This must be time variant, update by the rostopic "gpsSS"
            self.KF.Q = gps_covariance
            #z is the measurement from GPS, get it from rostopic "gps"
            self.KF.update(z = measurement)
        # print(f"estimation: {self.KF.x}")

    def odometryCallback(self, data):
        
        self.step += 1
        # Read radar odometry data from ros msg
        position = data.pose.pose.position
        odometry_covariance = np.array(data.pose.covariance).reshape(6, -1)[:2,:2]

        # Get euler angle from quaternion
        # Input of this fcn should be data from rostopic "radar_odometry.pose.pose.orientation"
        roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])

        # Calculate odometry difference
        # which is the prediction - the state of previous time frame
        diff = np.array([[position.x - self.last_odometry_position[0]],[position.y - self.last_odometry_position[1]]])
        diff_yaw = yaw - self.last_odometry_angle

        # KF predict
        if self.step == 1:
            #raise NotImplementedError
            self.init_KF(position.x, position.y, 0)
        else:
            #raise NotImplementedError
            #This must be time variant, update by the rostopic "radar_odometry"
            self.KF.R[:2,:2] += odometry_covariance
            # the input u means the difference between two state
            # it's calculated at line 62 and 63
            self.KF.predict(u = np.array([diff[0],diff[1],diff_yaw]))
        # print(f"estimation: {self.KF.x}")
        self.last_odometry_position = [position.x,position.y]
        print(self.KF.x[2][0])
        self.last_odometry_angle = self.KF.x[2][0]
        # change from euler to quaternion, the last input should be rotation about z axis
        # which is our self.x[2]
        quaternion = quaternion_from_euler(0, 0, yaw)
        # Publish odometry with covariance
        predPose = Odometry()
        predPose.header.frame_id = 'origin'
        predPose.pose.pose.position.x = self.KF.x[0]
        predPose.pose.pose.position.y = self.KF.x[1]
        predPose.pose.pose.orientation.x = quaternion[0]
        predPose.pose.pose.orientation.y = quaternion[1]
        predPose.pose.pose.orientation.z = quaternion[2]
        predPose.pose.pose.orientation.w = quaternion[3]
        predPose.pose.covariance = [self.KF.P[0][0], self.KF.P[0][1],0,0,0,0,
                                    self.KF.P[1][0], self.KF.P[1][1],0,0,0,0,
                                    0,0,0,0,0,0,
                                    0,0,0,0,0,0,
                                    0,0,0,0,0,0,
                                    0,0,0,0,0,0 ]
        self.posePub.publish(predPose)

    def gtCallback(self, data):
        gt_position = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        if self.KF is not None:
            kf_position = self.KF.x[:2]
        self.gt_list.append(gt_position)
        self.est_list.append(kf_position)

    def plot_path(self):
        plt.figure(figsize=(10, 8))
        plt.xlabel('x')
        plt.ylabel('y')
        plt.grid(True)
        gt_x, gt_y = zip(*self.gt_list)
        est_x, est_y = zip(*self.est_list)
        plt.plot(gt_x, gt_y, alpha=0.25, linewidth=8, label='Groundtruth path')
        plt.plot(est_x, est_y, alpha=0.5, linewidth=3, label='Estimation path')
        plt.title("KF fusion odometry result comparison")
        plt.legend()
        if not os.path.exists("/home/ee904/SDC/hw4/src/kalman_filter/results"):
            os.mkdir("/home/ee904/SDC/hw4/src/kalman_filter/results")
        plt.savefig("/home/ee904/SDC/hw4/src/kalman_filter/results/result.png")
        plt.show()

    def init_KF(self, x, y, yaw):
        # raise NotImplementedError
        # Initialize the Kalman filter when the first data comes in
        self.KF = KalmanFilter(x = x, y = y, yaw = yaw)
        # ????? 下面這兩行我不確定
        self.KF.A = np.identity(3)
        self.KF.B = np.identity(3)

if __name__ == '__main__':
    rospy.init_node('kf', anonymous=True)
    #raise NotImplementedError   
    fusion = Fusion()
    rospy.spin()
    # fusion.plot_path()
