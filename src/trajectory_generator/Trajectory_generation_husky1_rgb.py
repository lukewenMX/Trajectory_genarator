#! /usr/bin/env python

"""
Coded by Luke Wen on 14th Aug 2018.
The program is aiming to generate the trajectories for the detected objects

"""

import rospy
import tf
from tf import transformations,TransformListener
import message_filters

from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path,Odometry
from rgbdt_fusion.msg import Detection, DetectionFull, DetectionArray
from sensor_msgs.msg import PointCloud2,Image


import numpy as np
from scipy.spatial.distance import pdist


class MulTraj_Gene(object):
    """Generate the trajectory for the objects"""

    def __init__(self,node_name = "MulTraj_gene"):
        self.node_name = node_name
        rospy.init_node(self.node_name,anonymous="true")
        # self.pub = rospy.Publisher("trajectory",Path,queue_size = 2)
        self.info_sub = message_filters.Subscriber("/rgbdt_fusion/resColor/full",DetectionFull)
        self.odom_sub = message_filters.Subscriber("/husky1_robot_pose",Odometry)
        #self.odom_sub = message_filters.Subscriber("/optris/thermal_image",Image)
        self.paths_pub = []
        self.listener = TransformListener()
        self.paths = []
        self.paths_size = 0
        #self.pose = PoseStamped()
        self.trans_pose = PoseStamped()
        #self.pose_index = []
        self.velocity = []
        self.V_magnitude = []
        self.V_direction = []
        self.Human_list = []

    def info_odom_callback(self,info,odom):
        self.velocity[:] = []
        self.V_direction[:]=[]
        self.V_magnitude[:] = []
        if (info.detections.size > 0):

            #rospy.loginfo("%s",info.detections.size)
            # Store the path info for each detection and publish the path while getting the new message update.
            for entry in info.detections.data:
                if entry.num > self.paths_size:
                    for i in range(entry.num-self.paths_size):
                        self.paths.append(Path())
                        #self.dynamics.append(Dynamics())
                        self.paths_pub.append(rospy.Publisher("~trajectory"+str(self.paths_size + i + 1),Path,queue_size = 1))
                        #self.dynamics_pub.append(rospy.Publisher("~dynamics")+str(self.paths_size + i + 1),Dynamics,queue_size = 1)
                        #self.pose_index.append(0)
                    self.paths_size = entry.num

                pose = PoseStamped()
                point_stamped = entry.ptStamped
                pose.header = point_stamped.header
                pose.pose.position = point_stamped.point
                pose.pose.orientation.w = 1

                try:
					# Make true the frame_id of the pose (ptStamped)
                    self.trans_pose = self.listener.transformPose("map",pose)
                except(tf.Exception):
                    rospy.loginfo("The transformation is not available right now!")
                
                self.paths[entry.num - 1].poses.append(self.trans_pose)
                self.paths[entry.num - 1].header.frame_id = 'map'
                self.paths_pub[entry.num - 1].publish(self.paths[entry.num - 1])
                #self.pose_index[entry.num - 1] += 1
            #rospy.loginfo("published paths!")
            for index in range(self.paths_size):
                #self.velocity[:] = []
                if len(self.paths[index].poses) > 3:
                    num = index + 1
                    pre_x = self.paths[index].poses[-2].pose.position.x
                    pre_y = self.paths[index].poses[-2].pose.position.y
                    pre_z = self.paths[index].poses[-2].pose.position.z
                    pre_stamp = self.paths[index].poses[-2].header.stamp
                    cur_x = self.paths[index].poses[-1].pose.position.x
                    cur_y = self.paths[index].poses[-1].pose.position.y
                    cur_z = self.paths[index].poses[-1].pose.position.z
                    cur_stamp = self.paths[index].poses[-1].header.stamp
                    duration = (cur_stamp - pre_stamp).to_sec()
                    vx = (cur_x - pre_x) / duration 
                    vy = (cur_y - pre_y) / duration
                    self.velocity.append([cur_x,cur_y,cur_z,vx,vy,num])
                else:
                    pass 
            odom_x = odom.pose.pose.position.x 
            odom_y = odom.pose.pose.position.y 
            odom_z = odom.pose.pose.position.z
            #rospy.loginfo("There")
            '''odom_x = odom.height
            odom_y = odom.width
            odom_z = 1 '''
            self.info = self.velocity.append([odom_x,odom_y,odom_z]) # append
            #rospy.loginfo("The current velocity info are %s",self.velocity)
            if (len(self.velocity) > 1):
                Human_num = len(self.velocity) - 1
                for i in np.arange(Human_num):
                    self.Human_list.append([self.velocity[i][5]])
                    self.V_magnitude.append(np.sqrt(np.square(self.velocity[i][3]) + np.square(self.velocity[i][4])))
                    vector1 = [self.velocity[i][3], self.velocity[i][4]]
                    vector2 = [self.velocity[Human_num][0] - self.velocity[i][0], self.velocity[Human_num][1] - self.velocity[i][1]]
                    self.V_direction.append(np.dot(vector1, vector2)/(np.linalg.norm(vector1)*np.linalg.norm(vector2)))
                Unique_v = self.crf(self.V_magnitude)
                #print Unique_v
                #rospy.lginfo("HI")
                Unique_d = self.crf(self.V_direction)
                #print Unique_d
                Unique_d = self.normalize2(Unique_d)
                #print Unique_d
                #rospy.loginfo(len(self.V_direction))
                #rospy.loginfo(len(self.V_magnitude))
                Unique = self.normalize1(np.array(Unique_v) * np.array(Unique_d))
                Unique = list(Unique)
                #print Unique
                #print(len(self.velocity))
                #rospy.loginfo(Unique.index(max(Unique)))
                Unique_l = self.velocity[Unique.index(max(Unique))][5]
                rospy.loginfo("The Abnormal Object's label detected in husky1 camera is %s",Unique_l)
                #print("HELLO!")
        else:
            rospy.logdebug("HUSKY1 (camera):There is no person detected in the current frame!")

    def crf(self,a): # conditional random field in feature level
        m=[]
        for i in np.arange(np.size(a)):
            m.append(reduce(lambda x,y:x*y,a)/a[i])
        
        Z = reduce(lambda x,y:x+y,m)
        neibor = np.divide(m,Z)
        neibor = list(self.normalize1(np.divide(1,neibor)))
        #if np.var(neibor) <= 0.01 and np.var(neibor) != 0: # threshold for unique definition
        #    neibor=[]
        #print neibor
        return neibor

    def normalize1(self,a):
        if len(a) == 0:
            return a
        elif len(a) == 1:
            a = [1]
            return a
        
        sum = reduce(lambda x,y:x+y,a)
        for i in np.arange(np.shape(a)[0]):
            a[i] = a[i] / sum
        return a

    def normalize2(self,a):
        if min(a) < 0:
            for i in np.arange(len(a)):
                a[i] = (a[i]-min(a))/(max(a)-min(a))
        return a


        
if __name__ == "__main__":
    MulTraj_ge = MulTraj_Gene()
    #rospy.loginfo("The class has been initialized!")
    ts = message_filters.ApproximateTimeSynchronizer([MulTraj_ge.info_sub,MulTraj_ge.odom_sub],10,0.1)#,allow_headerless=True)
    #rospy.loginfo("Approximate Time!")
    ts.registerCallback(MulTraj_ge.info_odom_callback)
    #rospy.loginfo("Callback function!")
    #rospy.logerr(MulTraj_ge.pose.header.frame_id)
    #MulTraj_ge.listener.waitForTransform("map","thermal_camera",rospy.Time(),rospy.Duration(2.0))
    # while not rospy.is_shutdown():
    # while not rospy.is_shutdown():

    rospy.spin()

