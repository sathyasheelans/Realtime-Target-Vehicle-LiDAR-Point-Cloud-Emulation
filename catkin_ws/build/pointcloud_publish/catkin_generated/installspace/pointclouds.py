#!/usr/bin/env python3
import rospy
import math
import sys
import numpy as np
import open3d as o3d
import pickle
from math import floor, ceil

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose

import sensor_msgs.point_cloud2 as pcl2

class Augmented_PCL_Publish:

    def __init__(self,dict):
        rospy.init_node('augmented_pointcloud_pub', anonymous=True)
        self.rate=rospy.Rate(10)
        self.pose=Pose()
        self.curr_position=[]
        self.pcl_dict=dict
        self.pose_subscriber=rospy.Subscriber('target_pose',Pose,self.update_callback)
        rospy.spin()
        
        

    def update_callback(self,data):
        self.pose=data
        self.curr_position=[self.pose.position.x,self.pose.position.y,self.pose.position.z]
        self.print_pose()
        self.closest_point(self.curr_position)
        

    def print_pose(self):
        rospy.loginfo(self.curr_position)

    def closest_point(self, arr):
        f_x=floor(arr[0]) #Floor of x
        c_x=ceil(arr[0]) #Ceil of x
        f_y=floor(arr[1]) #Floor of y
        c_y=ceil(arr[1]) #Ceil of y
        f_z=floor(arr[2]) #Floor of z
        c_z=ceil(arr[2]) #Ceil of z
        closest_points=np.asarray([[f_x,f_y,f_z],
                        [f_x,c_y,f_z],
                        [f_x,c_y,c_z],
                        [f_x,f_y,c_z],
                        [c_x,c_y,c_z],
                        [c_x,c_y,f_z],
                        [c_x,f_y,c_z],
                        [c_x,f_y,f_z]])
        dist_2 = np.sum((closest_points - arr)**2, axis=1)
        #rospy.loginfo(closest_points[np.argmin(dist_2)])
        pcd=self.pcl_dict[list(closest_points[np.argmin(dist_2)],0,0,0)]
        rospy.loginfo(self.pcl_dict.keys())
        #return np.argmin(dist_2)
    
if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''

    with open('/home/santhanam.17/Carla_scripts/filename.pickle', 'rb') as handle:
        read_dict = pickle.load(handle)
    try:
        x=Augmented_PCL_Publish(read_dict)
        
        
    except rospy.ROSInterruptException:
        pass
    