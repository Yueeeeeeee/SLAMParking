#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
from filter import CarFilter

command_velocity = 0.0
command_steering = 0.0
actual_x_lidar = 0.0
actual_y_lidar = 0.0
actual_orientation_lidar = 0.0
actual_orientation_imu = 0.0
actual_velocity_driver = 0.0
correction_orientation_imu = 0.0
first_imu_received = False
first_lidar_received = False
correction_calculated = False
kf = CarFilter()

def imu_callback(data):
    global actual_orientation_imu
    global correction_orientation_imu
    global correction_calculated
    global first_imu_received
    global kf
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    w = data.orientation.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    if correction_calculated:
        actual_orientation_imu = yaw - correction_orientation_imu
        if actual_orientation_imu < 0:
            actual_orientation_imu =  actual_orientation_imu + 2 * math.pi
        elif actual_orientation_imu > 2 * math.pi:
            actual_orientation_imu = actual_orientation_imu - 2 * math.pi
        kf.IMUupdate(actual_orientation_imu)
    else:
        first_imu_received = True
        actual_orientation_imu = yaw
        if first_lidar_received:
            correction_orientation_imu = yaw - actual_orientation_lidar
            correction_calculated = True


def lidar_callback(data):
    global actual_x_lidar
    global actual_y_lidar
    global actual_orientation_lidar
    global first_lidar_received
    global kf
    # Get position from LIDAR and SLAM.
    actual_x_lidar = data.pose.pose.position.x
    actual_y_lidar = data.pose.pose.position.y
    # Get orientation from LIDAR and SLAM to ensure consistent frames.
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    actual_orientation_lidar = yaw
    first_lidar_received = True
    kf.Lidarupdate(actual_x_lidar, actual_y_lidar, actual_orientation_lidar)


def actual_velocity_callback(data):
    global actual_velocity_driver
    actual_velocity_driver = data.data
    kf.AVupdate(actual_velocity_driver)


def command_velocity_callback(data):
    global command_velocity
    command_velocity = data.data


def command_steering_callback(data):
    global command_steering
    command_steering = data.data


def main():
    global kf
    pub = rospy.Publisher('pose_filtered', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('kalman_filter')
    rospy.Subscriber('imu/data', Imu, imu_callback)
    rospy.Subscriber('poseupdate', PoseWithCovarianceStamped, lidar_callback)
    rospy.Subscriber('actual_velocity', Float32, actual_velocity_callback)
    rospy.Subscriber('velocity', Float32, command_velocity_callback)
    rospy.Subscriber('steering', Float32, command_steering_callback)
    rate = rospy.Rate(100) # 100 Hz
    delta_t = 0.1
    while not rospy.is_shutdown():
        kf.predict(delta_t, command_velocity, command_steering)
        msg = kf.get_state_as_message()
        pub.publish(msg)
        #rospy.loginfo('Kalman filter orientation = %s', msg.pose.pose.orientation.w)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
