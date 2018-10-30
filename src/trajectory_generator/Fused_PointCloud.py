#! /usr/bin/env python

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
import math

#from  sensor_msgs.msg  import PointCloud
from rgbdt_fusion.msg import Detection, DetectionArray, DetectionFull
# from sensor_msgs.msg import PointCloud
#from geographic_msgs.msg import PointStamped
from tf import transformations,TransformListener


class Fused_PointCloud(object):
    "The init function for the fused detectionArray"
    def __init__(self,node_name="fused_detectionArray"):
        self.node_name = node_name
        rospy.init_node(self.node_name)
        #self.fused_pointcloud = PointCloud()
        self.listener = TransformListener()
        self.R = 100
        self.angle = 30
        self.h = self.R * math.cos(self.angle/180 * math.pi)
        self.r = self.R * math.sin(self.angle/180 * math.pi)
        # self.DetectionInfo_sub1 = message_filters.Subscriber("rgbdt_fusion/resColor/full", DetectionFull)
        # self.DetectionInfo_sub2 = message_filters.Subscriber("rgbdt_fusion/resColor/full2", DetectionFull)
        self.DetectionInfo_sub1 = rospy.Subscriber("rgbdt_fusion/resColor/full", DetectionFull, self.sub1_callback)
        self.DetectionInfo_sub2 = rospy.Subscriber("rgbdt_fusion/resColor/full2", DetectionFull, self.sub2_callback)
        self.DetectionInfo_pub = rospy.Publisher('fused_detectionarray',DetectionArray,queue_size=2)
        self.sub1_new = False
        self.sub2_new = False
        self.storeDetectionInfo1 = None
        self.storeDetectionInfo2 = None

    
    def sub1_callback(self, DetectionInfo):
        self.storeDetectionInfo1 = DetectionInfo
        self.sub1_new = True

    def sub2_callback(self, DetectionInfo):
        self.storeDetectionInfo2 = DetectionInfo
        self.sub2_new = True

    def DetectionInfo(self):
        DetectionInfo1 = self.storeDetectionInfo1.detections
        DetectionInfo2 = self.storeDetectionInfo2.detections
        detect_data1_map = self.Transfer_global_map(DetectionInfo1,1)
        detect_data2_map = self.Transfer_global_map(DetectionInfo2,2)
        detect_fused_data_map = self.Fuse_by_distance(detect_data1_map,detect_data2_map,distance_threshold = 1.0, color_threshold = 30) # need to modify the color threshold
        rospy.loginfo("COMES")
        self.DetectionInfo_pub.publish(detect_fused_data_map)
        self.sub1_new = False
        self.sub2_new = False

        
    def Transfer_global_map(self,Detect_data,index):# The input is message with type of DetectionArray
        Detect_data_ = DetectionArray()
        # print(Detect_data_)
        Detect_data_= Detect_data
        #print(Detect_data)
        # Try reproject the stamped point to the global map
        if Detect_data.size > 0:
            for i in range(Detect_data.size):
                print(i)
                try: 
                    Detect_data_.data[i].ptStamped = self.listener.transformPoint("map",Detect_data.data[i].ptStamped) # Potential problem is from the time stamp
                except tf.Exception:
                    rospy.loginfo("Some tf Exception happened!")
        rospy.loginfo("The transformation is done currently!"+str(index))

        return Detect_data_

    def Fuse_by_distance(self,detect_data1,detect_data2,distance_threshold = 1.0,color_threshold = 30): # The inputs here are both with type of DetectionArray
        detect_data = detect_data1
        # print("detect_data.size before: ")
        # print(detect_data.size)
        size = detect_data2.size
        # value = self.HSV_distance(detect_data1.data[i].color[0],detect_data1.data[i].color[1],detect_data1.data[i].color[2],detect_data2.data[j].color[0],detect_data2.data[j].color[1].detect_data2.data[j].color[2])
        # print(type(detect_data2.data))
        for i in range(detect_data1.size):
            for j in range(detect_data2.size):
                if (math.sqrt(pow(detect_data1.data[i].ptStamped.point.x - detect_data2.data[j].ptStamped.point.x,2) + pow(detect_data1.data[i].ptStamped.point.y - detect_data2.data[j].ptStamped.point.y,2) \
                + pow(detect_data1.data[i].ptStamped.point.z - detect_data2.data[j].ptStamped.point.z,2)) < distance_threshold) and (self.HSV_distance(detect_data1.data[i].color[0],detect_data1.data[i].color[1],detect_data1.data[i].color[2],detect_data2.data[j].color[0],detect_data2.data[j].color[1].detect_data2.data[j].color[2]) < color_threshold):
                    print(type(detect_data2.data))
                    detect_data2.data.pop[j]
                    print("pop out an element!")
                    size -= 1
        # print("detect_data2")
        # print(detect_data2)
        # print("detect_data2 left humans: ")
        # print(size)
        for i in range(size):
            detect_data.data.append(detect_data2.data[i]) # There might be a problem when appending the pointstamped with different time stamped
            if detect_data.size > 1:           
                detect_data.data[-1].num = detect_data.data[-2].num + 1
            else:
                pass
            detect_data.size += 1
        # print("detect_data.size after: ")
        # print(detect_data.size)
        # print("detect_data")
        # print(detect_data)
        

        return detect_data


    # reference link : 
    def HSV_distance(self,h1,s1,v1,h2,s2,v2):
        x1 = self.r * v1 * s1 * math.cos(h1 / 180 * math.pi)
        y1 = self.r * v1 * s1 * math.sin(h1 / 180 * math.pi)
        z1 = self.h * (1 - v1)

        x2 = self.r * v2 * s2 * math.cos(h2 / 180 * math.pi)
        y2 = self.r * v2 * s2 * math.sin(h2 / 180 * math.pi)
        z2 = self.h * (1 - v2)

        dx,dy,dz = x1 - x2,y1 - y2, z1 - z2
        return math.sqrt(dx * dx + dy * dy + dz * dz) 



if __name__ == "__main__":
    Fused_pointcloud = Fused_PointCloud()
    print("Main")
    #rospy.loginfo("The class has been initialized!")
    # ts = message_filters.ApproximateTimeSynchronizer([Fused_pointcloud.DetectionInfo_sub1,Fused_pointcloud.DetectionInfo_sub2],10,0.1,allow_headerless=True)
    # rospy.loginfo("Approximate Time!")
    # ts.registerCallback(Fused_pointcloud.DetectionInfo)
    # rospy.loginfo("Callback function!")
    #rospy.logerr(MulTraj_ge.pose.header.frame_id)
    #MulTraj_ge.listener.waitForTransform("map","thermal_camera",rospy.Time(),rospy.Duration(2.0))
    # while not rospy.is_shutdown():
    while not rospy.is_shutdown():
        if Fused_pointcloud.sub1_new and Fused_pointcloud.sub2_new:
            Fused_pointcloud.DetectionInfo()

    rospy.spin()









##### before add color

# #! /usr/bin/env python

# """
# Coded by Luke Wen on 16th Oct 2018
# This program is aiming to fused the human-points estimated from husky robots, where the human detected will be fused as on if their respect distance
# is within the threshold which we set as 1m, otherwise the obejects are considered as seperate ones

# Input: the two DetectionFull message
# Output: fused pointcloud
# """

# import rospy 
# import message_filters
# import tf
# import math

# #from  sensor_msgs.msg  import PointCloud
# from rgbdt_fusion.msg import Detection, DetectionArray, DetectionFull
# # from sensor_msgs.msg import PointCloud
# #from geographic_msgs.msg import PointStamped
# from tf import transformations,TransformListener


# class Fused_PointCloud(object):
#     "The init function for the fused detectionArray"
#     def __init__(self,node_name="fused_detectionArray"):
#         self.node_name = node_name
#         rospy.init_node(self.node_name)
#         #self.fused_pointcloud = PointCloud()
#         self.listener = TransformListener()
#         self.DetectionInfo_sub1 = message_filters.Subscriber("rgbdt_fusion/resColor/full", DetectionFull)
#         self.DetectionInfo_sub2 = message_filters.Subscriber("rgbdt_fusion/resColor/full2", DetectionFull)
#         self.DetectionInfo_pub = rospy.Publisher('fused_detectionarray',DetectionArray,queue_size=2)

#     def DetectionInfo(self, DetectionInfo1, DetectionInfo2):
#         DetectionInfo1 = DetectionInfo1.detections
#         DetectionInfo2 = DetectionInfo2.detections
#         detect_data1_map = self.Transfer_global_map(DetectionInfo1,1)
#         detect_data2_map = self.Transfer_global_map(DetectionInfo2,2)
#         detect_fused_data_map = self.Fuse_by_distance(detect_data1_map,detect_data2_map,threshold = 1.0)
#         rospy.loginfo("COMES")
#         self.DetectionInfo_pub.publish(detect_fused_data_map)

        
#     def Transfer_global_map(self,Detect_data,index):# The input is message with type of DetectionArray
#         Detect_data_ = DetectionArray()
#         # print(Detect_data_)
#         Detect_data_ = Detect_data
#         print(Detect_data_)
#         # Try reproject the stamped point to the global map
#         if Detect_data.size > 0:
#             for i in range(Detect_data.size):
#                 # print(i)
#                 try: 
#                     Detect_data_.data[i].ptStamped = self.listener.transformPoint("map",Detect_data.data[i].ptStamped) # Potential problem is from the time stamp
#                     print(i)
#                 except tf.Exception:
#                     rospy.loginfo("Some tf Exception happened!")
#         rospy.loginfo("The transformation is done currently!"+str(index))

#         return Detect_data_

#     def Fuse_by_distance(self,detect_data1,detect_data2,threshold = 1.0): # The inputs here are both with type of DetectionArray
#         detect_data = detect_data1
#         print("detect_data.size before: ")
#         print(detect_data.size)
#         size = detect_data2.size
#         print(type(detect_data2.data))
#         for i in range(detect_data1.size):
#             for j in range(detect_data2.size):
#                 if math.sqrt(pow(detect_data1.data[i].ptStamped.point.x - detect_data2.data[j].ptStamped.point.x,2) + pow(detect_data1.data[i].ptStamped.point.y - detect_data2.data[j].ptStamped.point.y,2) + pow(detect_data1.data[i].ptStamped.point.z - detect_data2.data[j].ptStamped.point.z,2)) < threshold:
#                     print(type(detect_data2.data))
#                     detect_data2.data.pop[j]
#                     size -= 1
#         print("detect_data2")
#         print(detect_data2)
#         print("detect_data2 left humans: ")
#         print(size)
#         for i in range(size):
#             detect_data.data.append(detect_data2.data[i]) # There might be a problem when appending the pointstamped with different time stamped
#             detect_data.size += 1
#         # print("detect_data.size after: ")
#         # print(detect_data.size)
#         print("detect_data")
#         print(detect_data)
        

#         return detect_data

# if __name__ == "__main__":
#     Fused_pointcloud = Fused_PointCloud()
#     #rospy.loginfo("The class has been initialized!")
#     ts = message_filters.ApproximateTimeSynchronizer([Fused_pointcloud.DetectionInfo_sub1,Fused_pointcloud.DetectionInfo_sub2],10,0.1,allow_headerless=True)
#     # rospy.loginfo("Approximate Time!")
#     ts.registerCallback(Fused_pointcloud.DetectionInfo)
#     # rospy.loginfo("Callback function!")
#     #rospy.logerr(MulTraj_ge.pose.header.frame_id)
#     #MulTraj_ge.listener.waitForTransform("map","thermal_camera",rospy.Time(),rospy.Duration(2.0))
#     # while not rospy.is_shutdown():
#     # while not rospy.is_shutdown():

#     rospy.spin()