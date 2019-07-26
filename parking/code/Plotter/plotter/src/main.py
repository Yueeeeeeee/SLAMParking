#!/usr/bin/env python
import math
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

pub = rospy.Publisher('path', Path, queue_size=10)
path = Path()

def pose_callback(data):
    global path
    global pub
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    if len(path.poses) > 0:
        prev = path.poses[-1].position
        dist = (pose.pose.position.x - prev.x)**2 + (pose.pose.position.y - prev.y)**2
        # if distance larger 4 cm
        if dist > (0.04*0.04):
            path.poses.append(pose)
            # limit path length
            if len(path.poses) > 1000:
                path.poses.pop(0)
            pub.publish(path)
    else:    
        path.poses.append(pose)
        pub.publish(path)


def main():
    rospy.init_node('plotter')
    # /poseupdate from Hector SLAM
    #rospy.Subscriber('poseupdate', PoseWithCovarianceStamped, pose_callback)
    # /pose_filtered from Kalman Filter
    rospy.Subscriber('pose_filtered', PoseWithCovarianceStamped, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
