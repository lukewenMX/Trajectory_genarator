#! /usr/bin/env python

"""
Testing the position and velocity (x,y,z,vx,vy,vz) input and show the pose info in rviz on the map.

"""

import rospy
from geometry_msgs.msg import PoseStamped,Quaternion,Twist
import tf
from tf import transformations

class Pose_generation(object):
    def __init__(self,node_name="pose_generation"):
        rospy.init_node(node_name)
        self.info_sub = rospy.Subscriber("info",Twist,self.info_callback)
        self.pub = rospy.Publisher("object_pose",PoseStamped,queue_size =2)
        self.object_pose = PoseStamped()
    def info_callback(self,info):
        self.object_pose.pose.position.x = info.linear.x
        self.object_pose.pose.position.y = info.linear.y
        self.object_pose.pose.position.z = info.linear.z
        quaternion = transformations.quaternion_from_euler(info.angular.x,info.angular.y,info.angular.z)
        self.object_pose.pose.orientation.x = quaternion[0]
        self.object_pose.pose.orientation.y = quaternion[1]
        self.object_pose.pose.orientation.z = quaternion[2]
        self.object_pose.pose.orientation.w = quaternion[3]
        self.object_pose.header.stamp = rospy.Time.now()
        self.object_pose.header.frame_id = "map"
        self.pub.publish(self.object_pose)
        rospy.loginfo("The object pose has been published !")


if __name__=="__main__":
    pose_generation = Pose_generation("pose_generation")
    #rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()
        
    



