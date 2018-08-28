#! /usr/bin/env python

"""
Coded by Luke Wen on 14th Aug 2018.
The program is aiming to generate the trajectories for the detected objects

"""

import rospy
import tf
from tf import transformations,TransformListener
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from rgbdt_fusion.msg import Detection, DetectionFull, DetectionArray


class MulTraj_Gene(object):
    """Generate the trajectory for the objects"""
    def __init__(self,node_name = "MulTraj_gene"):
        self.node_name = node_name
        rospy.init_node(self.node_name)
        # self.pub = rospy.Publisher("trajectory",Path,queue_size = 2)
        self.sub = rospy.Subscriber("/rgbdt_fusion/resColor/full",DetectionFull,self.info_callback)
        self.paths_pub = []
        self.listener = TransformListener()
        self.paths = []
        self.paths_size = 0
        #self.pose = PoseStamped()
        self.trans_pose = PoseStamped()
    def info_callback(self,info):
        if (info.detections.size > 0):
            #rospy.loginfo("%s",info.detections.size)
            for entry in info.detections.data:
                if entry.num > self.paths_size:
                    for i in range(entry.num-self.paths_size):
                        self.paths.append(Path())
                        self.paths_pub.append(rospy.Publisher("~trajectory"+str(self.paths_size+i+1),Path,queue_size = 1))
                    self.paths_size = entry.num

                pose = PoseStamped()
                point_stamped = entry.ptStamped
                pose.header = point_stamped.header
                pose.pose.position = point_stamped.point
                pose.pose.orientation.w = 1

                try:
                    self.trans_pose = self.listener.transformPose("map",pose)
                except(tf.Exception):
                    rospy.loginfo("The transformation is not available right now!")
                
                self.paths[entry.num-1].poses.append(self.trans_pose)
                self.paths[entry.num-1].header.frame_id = 'map'
                if len(self.paths[entry.num - 1].poses) > 3:
                    self.paths_pub[entry.num-1].publish(self.paths[entry.num - 1])
                '''paths[num].poses.append(pose)
                
                if info.detections.data[i].num == 1:
                    #rospy.loginfo("hi!")
                    pose = PoseStamped()
                    point_stamped = info.detections.data[i].ptStamped
                    pose.header = point_stamped.header
                    #rospy.loginfo(self.pose.header)
                    pose.pose.position = point_stamped.point
                    pose.pose.orientation.w = 1
                    try:
                         #self.listener.waitForTransform("map","thermal_camera",rospy.Time.now(),rospy.Duration(2.0))
                        self.trans_pose = self.listener.transformPose("map",pose)
                        self.path.poses.append(self.trans_pose)
                        self.pub.publish(self.path)
                    except(tf.Exception,rospy.exceptions.ROSTimeMovedBackwardsException):
                        rospy.loginfo("The tranformation is not available right now!")
                        pass
                # else:
                #     rospy.loginfo("No Person Detected!")
                #     pass
            #pose = PoseStamped

            for i in range(paths.size):
                if (paths[i].size > 2):
                    paths_pub[i].publish()
            '''        
        else:
            rospy.logdebug("There is no person detected in the current frame!")

if __name__ == "__main__":
    MulTraj_ge = MulTraj_Gene()
    #rospy.logerr(MulTraj_ge.pose.header.frame_id)
    #MulTraj_ge.listener.waitForTransform("map","thermal_camera",rospy.Time(),rospy.Duration(2.0))
    # while not rospy.is_shutdown():
    rospy.spin()

    
