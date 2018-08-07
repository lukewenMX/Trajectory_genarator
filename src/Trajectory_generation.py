#! /usr/bin/env python
"""
Coded by Luke Wen on 6th Aug 2018.
The program is aiming to generate the trajectory info from the (x,y,z) positions of the central point of bounding box.
The input: odom generated from the position infomation with respect to the map .
Output: the trajectory.

"""

import rospy
#import tf
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
#from geometry_msgs.msg import Point,Pose,Quaternion,Vector3,Twist
from geometry_msgs.msg import Point, PoseStamped

path  = Path()

# the callback function for the odometry
def odom_callback(data):
    global path 
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)


if __name__=="__main__":
    # define the node name 
    rospy.init_node("Trajectory_Generator")
    odom_sub = rospy.Subscriber('/odometry/filtered',Odometry,odom_callback)
    path_pub = rospy.Publisher('path_generated',Path,queue_size = 3)

    while not rospy.is_shutdown():
        rospy.spin()







