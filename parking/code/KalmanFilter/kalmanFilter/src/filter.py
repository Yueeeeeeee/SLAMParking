import rospy
import numpy as np
from numpy.linalg import inv
import math
from geometry_msgs.msg import PoseWithCovarianceStamped

class CarFilter:
    def __init__(self):
        initial_confidence = 0.1
        initial_unsure = 10000.0
        # Initial state covariance
        self.P = np.array([(initial_confidence, 0, 0, 0, 0, 0),
                           (0, initial_confidence, 0, 0, 0, 0),
                           (0, 0, initial_unsure,     0, 0, 0),
                           (0, 0, 0, initial_confidence, 0, 0),
                           (0, 0, 0, 0, initial_confidence, 0),
                           (0, 0, 0, 0, 0, initial_confidence)])

        # initial state
        self.x = np.zeros(6)
        # process noise covariance
        self.Q = np.eye(6) * 0.03**2 + np.ones((6, 6)) * (0.01)**2 # no clue...
        # sensor noise covariance
        self.R_l = np.eye(3) * 0.05**2 # assume 5 cm error
        self.R_i = (0.05 * math.pi * 2 / 360)**2 # assume 0.05 error
        self.R_a = 0.0063 # measured

    def predict(self, delta_t, velocity, steering):
        one_div_wheelbase = 1 / 0.32
        # compute new state vector
        # x_k|k-1 = f(x_k-1|k-1, u_k)
        x_new = np.zeros(6)
        x_new[0] = delta_t * self.x[4] + self.x[0] # x = dt * v_x + x
        x_new[1] = delta_t * self.x[5] + self.x[1]
        x_new[2] = delta_t * one_div_wheelbase * math.tan(self.x[3]) * velocity + self.x[2]
        x_new[3] = steering
        x_new[4] = math.cos(self.x[2]) * velocity * delta_t
        x_new[5] = math.sin(self.x[2]) * velocity * delta_t
        # compute Jacobian of f with respect to x
        F = np.array([(1, 0, 0, 0, delta_t, 0),
              (0, 1, 0, 0, 0, delta_t),
              (0, 0, 1, delta_t * one_div_wheelbase * 1/math.cos(self.x[3])**2 * velocity, 0, 0),
              (0, 0, 0, 1, 0, 0),
              (0, 0, -math.sin(self.x[2])*velocity*delta_t, 0, 0, 0),
              (0, 0, math.cos(self.x[2])*velocity*delta_t, 0, 0, 0)])
        # compute new covariance matrix
        # P_k|k-1 = F_k*P_k-1|k-1*F_k^T+Q_k
        P_new = F.dot(self.P).dot(F.T) + self.Q
        # assign new values to state variables
        self.x = x_new
        self.P = P_new

    def Lidarupdate(self, x_LIDAR, y_LIDAR, orientation_LIDAR):
          # Sensor readings
        z_l = np.zeros(3)
        z_l[0] = x_LIDAR # x from LIDAR
        z_l[1] = y_LIDAR # y from LIDAR
        z_l[2] = orientation_LIDAR # orientation from LIDAR
        # measurement residual
        # y_k = z_k - h(x_k|k-1)
        y_l = np.zeros(3)
        y_l[0] = z_l[0] - self.x[0]
        y_l[1] = z_l[1] - self.x[1]
        y_l[2] = z_l[2] - self.x[2]
        # Jacobian of h with respect to x
        H_l = np.array([(1, 0, 0, 0, 0, 0),
                      (0, 1, 0, 0, 0, 0),
                      (0, 0, 1, 0, 0, 0)])                 
        # S_k = H_k * P_k|k-1 * H_k^T + R_k
        S_l = H_l.dot(self.P).dot(H_l.T) + self.R_l
        # Kalman gain
        # K_k = P_k|k-1 * H_k^T * S_k^-1
        K_l = self.P.dot(H_l.T).dot(inv(S_l))
        # new state vector
        # x_k|k = x_k|k-1 + K_k * y_k
        x_new = self.x + K_l.dot(y_l)
        # P_k|k = (I - K_k * H_k) * P_k|k-1
        P_new = (np.eye(6) - K_l.dot(H_l)).dot(self.P)
        # assign state variables
        self.x = x_new
        self.P = P_new

    def IMUupdate(self,orientation_IMU):
        # Sensor readings
        z_i = np.zeros(1)
        z_i[0] = orientation_IMU # orientation from IMU
        # measurement residual
        # y_k = z_k - h(x_k|k-1)
        y_i = np.zeros(1)
        y_i[0] = z_i[0] - self.x[2] # intentional
        # Jacobian of h with respect to x

        H_i = np.array([(0, 0, 1, 0, 0, 0)])
        # S_k = H_k * P_k|k-1 * H_k^T + R_k
        S_i = H_i.dot(self.P).dot(H_i.T) + self.R_i
        # Kalman gain
        # K_k = P_k|k-1 * H_k^T * S_k^-1
        K_i = self.P.dot(H_i.T).dot(inv(S_i))
        # new state vector
        # x_k|k = x_k|k-1 + K_k * y_k
        x_new = self.x + K_i.dot(y_i)
        # P_k|k = (I - K_k * H_k) * P_k|k-1
        P_new = (np.eye(6) - K_i.dot(H_i)).dot(self.P)
        # assign state variables
        self.x = x_new
        self.P = P_new
        
    def AVupdate(self, velocity_MOTOR):
        # Sensor readings
        z_a = np.zeros(1)

        z_a[0] = velocity_MOTOR # velocity from motor driver
        # measurement residual
        # y_k = z_k - h(x_k|k-1)
        y_a = np.zeros(1)
   
        y_a[0] = z_a[0] - math.sqrt(self.x[4]**2 + self.x[5]**2)
        # Jacobian of h with respect to x
        denom = math.sqrt(self.x[4]**2 + self.x[5]**2)
        if math.fabs(self.x[4]) > 0.000001:
            dv_by_dx = self.x[4] / denom
        else:
            dv_by_dx = 0.0
        if math.fabs(self.x[5]) > 0.000001:
            dv_by_dy = self.x[5] / denom
        else:
            dv_by_dy = 0.0
        H_a = np.array([
                      (0, 0, 0, 0, dv_by_dx, dv_by_dy)])
        # S_k = H_k * P_k|k-1 * H_k^T + R_k
        S_a = H_a.dot(self.P).dot(H_a.T) + self.R_a
        # Kalman gain
        # K_k = P_k|k-1 * H_k^T * S_k^-1
        K_a = self.P.dot(H_a.T).dot(inv(S_a))
        # new state vector
        # x_k|k = x_k|k-1 + K_k * y_k
        x_new = self.x + K_a.dot(y_a)
        # P_k|k = (I - K_k * H_k) * P_k|k-1
        P_new = (np.eye(6) - K_a.dot(H_a)).dot(self.P)
        # assign state variables
        self.x = x_new
        self.P = P_new
        
    def get_state_as_message(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = self.x[0]
        msg.pose.pose.position.y = self.x[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.x[2] / 2)
        msg.pose.pose.orientation.w = math.cos(self.x[2] / 2)
        return msg
