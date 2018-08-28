#! /usr/bin/env python
"""
Coded by Luke Wen on 12th Aug 2018.
The program is aiming to re-project the 3d point into the robot 1 local map.
The input: odom generated from the position infomation with respect to the map .
Output: the local map assosiated with the 3D point.

"""

import rospy 
import tf
from tf import TransformListener 
#from nav_msgs.msg import Path
#from geometry_msgs.msg import Point,Pose,Quaternion,Vector3,Twist
from geometry_msgs.msg import PointStamped
from rgbdt_fusion.msg import DetectionFull,Detection,DetectionArray


class Trajectory_generator(object):
    # Generate the object's trajectory from the central point of the detected box, then project
    def __init__(self,node_name="point_reproject"):
        self.point = PointStamped()
        self.node_name = node_name
        #self.listener = tf.TransformListener()
        rospy.init_node(self.node_name)
        self.tf = TransformListener()
        self.point_sub = rospy.Subscriber("/rgbdt_fusion/resColor/full",DetectionFull,self.Point_callback)
        self.point_pub = rospy.Publisher("point_reproject",PointStamped,queue_size = 2)

    def Point_callback(self,detectionsfull):
        #self.listener.waitForTransform("/map","/velodyne",rospy.Time(),rospy.Duration(0.5))
        #try:
        #    now = rospy.Time.now()
        #    self.listener.waitForTransform("/map","/velodyne",now,rospy.Duration(0.5))
        #    (trans,rot) = listener.lookupTransform("/map","/velodyne",now)
        time = rospy.Time.now()
        rospy.logerr(time)
        if (detectionsfull.detections.size >= 1):
        	for i in range(detectionsfull.detections.size):
        		if detectionsfull.detections.data[i].object_class == "person":	
        			
        			point_stamped = detectionsfull.detections.data[i].ptStamped
        			#self.tf.waitForTransform("base_link", "thermal_camera", rospy.Time.now(), rospy.Duration(4.0))
        			try:
        				now = rospy.Time.now()
        				self.tf.waitForTransform("map","thermal_camera", now,rospy.Duration(4.0))
        				
		        		point_transformed = self.tf.transformPoint("map",point_stamped)
		        		self.point = point_transformed
		        		'''self.path.header = point_transformed.header
		        		pose = PoseStamped()
		        		pose.header = point_transformed.header
		        		pose.pose.position.x = point_transformed.point.x
		        		pose.pose.position.y = point_transformed.point.y
		        		pose.pose.position.z = point_transformed.point.z
        				pose.pose.orientation.x = 0
        				pose.pose.orientation.y = 0
        				pose.pose.orientation.z = 0
        				pose.pose.orientation.w = 1
        				self.path.poses.append(pose)'''
        				self.point_pub.publish(self.point)
        				rospy.loginfo("The pointhas been published!")
        					#break
    				except(tf.ConnectivityException,tf.LookupException,tf.ExtrapolationException,rospy.ROSTimeMovedBackwardsException):
    						rospy.loginfo("Some Exceptions happend!")
        else:
        	
     		rospy.logdebug("There is no detection of person available!")
if __name__ == "__main__":
    trajectory_generate = Trajectory_generator(node_name = "Traj_gene_node")

    while not rospy.is_shutdown():
       rospy.spin()








