#! usr/bin/env python

"""
Coded by Luke Wen on 16th Oct 2018
This program is aiming to fused the human-points estimated from husky robots, where the human detected will be fused as on if their respect distance
is within the threshold which we set as 1m, otherwise the obejects are considered as seperate ones

Input: the two DetectionFull message
Output: fused pointcloud
"""

import rospy 
import message_filters
import tf

from  sensor_msgs.msg  import PointCloud
from rgbdt_fusion.msg import Detection, DetectionArray, DetectionFull
# from sensor_msgs.msg import PointCloud
from geographic_msgs.msg import PointStamped



class Fused_PointCloud(object):
    "The init function for the fused pointcloud"
    def __init__(self,node_name="fused_pointcloud"):
        self.node_name = node_name
        self.fused_pointcloud = PointCloud()
        rospy.init_node(self.node_name)
        self.DetectionInfo_sub1 = message_filters.Subscriber("rgbdt_fusion/resColor/full", DetectionFull)
        self.DetectionInfo_sub2 = message_filters.Subscriber("rgbdt_fusion/resColor/full2", DetectionFull)
        self.pointcloud_pub = rospy.Publisher('fused_pointcloud',PointCloud,queue_size=2)

    def DetectionInfo_callback(self, DetectionInfo1, DetectionInfo2):
        DetectionInfo = DetectionFull()
        DetectionInfo = DetectionInfo1

    def Transfer_global_map(self,Detect_data):
        Detect_data_ = Detection()
        Detect_data_ = Detect_data
        
        return pointcloud

    def Fuse_by_distance(self,pc1,pc2)



        
            

        




