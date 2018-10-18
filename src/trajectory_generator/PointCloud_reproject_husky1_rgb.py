#! /usr/bin/env python
"""
Coded by Luke Wen on 12th Aug 2018.
The program is aiming to re-project the 3d point into the robot 1 local map.
The input: odom generated from the position infomation with respect to the map .
Output: the local map assosiated with the 3D point.

"""

import rospy 
import tf
from tf import TransformListener,TransformerROS
#from nav_msgs.msg import Path
#from geometry_msgs.msg import Point,Pose,Quaternion,Vector3,Twist
from geometry_msgs.msg import PointStamped
#from rgbdt_fusion.msg import DetectionFull,Detection,DetectionArray
from sensor_msgs.msg import PointCloud,PointCloud2

class Trajectory_generator(object):
    # Generate the object's trajectory from the central point of the detected box, then project
    def __init__(self,node_name="pointcloud_reproject"):
        self.pointcloud = PointCloud()
        self.node_name = node_name
        #self.listener = tf.TransformListener()
        rospy.init_node(self.node_name)
        self.tf = TransformListener()
        #self.tf = TransformerROS()
        self.pointcloud_sub = rospy.Subscriber("/output_cloud",PointCloud,self.Pointcloud_callback)
        self.pointcloud_pub = rospy.Publisher("~pointcloud_reproject",PointCloud,queue_size = 2)

    def Pointcloud_callback(self,human_centers):
        #self.listener.waitForTransform("/map","/velodyne",rospy.Time(),rospy.Duration(0.5))
        #try:
        #    now = rospy.Time.now()
        #    self.listener.waitForTransform("/map","/velodyne",now,rospy.Duration(0.5))
        #    (trans,rot) = listener.lookupTransform("/map","/velodyne",now)
        #time = rospy.Time.now()
        #rospy.logerr(time)
		#self.tf.waitForTransform("map","thermal_camera",rospy.Time.now(),rospy.Duration(2.0))
		#rospy.loginfo(human_centers.header)
        try:
            #rospy.loginfo("Here!")
            header = human_centers.header
            #human_centers.header.frame_id = "husky2/left_camera"
            human_centers.header.stamp = rospy.Time.now()
            self.tf.waitForTransform("map","left_camera",rospy.Time.now(),rospy.Duration(1.0))
            self.pointcloud = self.tf.transformPointCloud("map",human_centers)
            self.pointcloud.header.stamp = header.stamp
            #rospy.loginfo(self.pointcloud.header)
            # rospy.loginfo(human_centers.header)
            self.pointcloud_pub.publish(self.pointcloud)
        except(tf.LookupException,tf.ConnectivityException):
            rospy.loginfo("There are any exception happened!")#,tf.ExtrapolationException,rospy.ROSTimeMovedBackwardsException):
        rospy.loginfo("Husky1: The point has been published!")
                
        #print("Exit")
if __name__ == "__main__":

    trajectory_generate = Trajectory_generator(node_name = "Traj_gene_node")
    #while not rospy.is_shutdown():
        #rospy.loginfo("well done")
    rospy.spin()
        
    
       