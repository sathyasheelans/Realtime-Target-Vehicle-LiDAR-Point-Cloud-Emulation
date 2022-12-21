#!/usr/bin/env python
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
        self.point_background=PointCloud2()
        self.curr_position=[]
        self.pcl_dict=dict
        self.pose_subscriber=rospy.Subscriber('target_pose',Pose,self.update_callback)
        #self.points_raw_subscriber=rospy.Subscriber('points_raw',PointCloud2,self.point_cloud_subscribe)
        rospy.spin()
        
    def update_callback(self,data):
        self.pose=data
        self.curr_position=np.array([self.pose.position.x,self.pose.position.y,self.pose.position.z])
        #self.print_pose()
        #self.closest_point(self.curr_position)
        self.point_cloud_publish(self.closest_point(self.curr_position))
    
    def print_pose(self):
        rospy.loginfo(self.curr_position)

    def closest_point(self, arr):
        Ego_postion=np.array([0,0,0])
        dist = round(np.linalg.norm(arr - Ego_postion))
        keys=list(self.pcl_dict.keys())
        closest_keys=[k for k in self.pcl_dict.keys() if k[3]==dist]
        closest_points=np.array([[key[0],key[1],key[2]] for key in closest_keys])
        dist_2 = np.sum((closest_points - arr)**2, axis=1)
        #rospy.loginfo(closest_points[np.argmin(dist_2)])
        closest_key=closest_points[np.argmin(dist_2)]
        closest_key=np.append(closest_key,dist)
        pcd=self.pcl_dict[tuple(list(closest_key))]
        
        """rospy.loginfo(np.linalg.norm(arr - Ego_postion))
        rospy.loginfo(dist)
        rospy.loginfo(tuple(list(closest_key)))
        rospy.loginfo(pcd)"""
        #rospy.loginfo(len(closest_points))
        return pcd

    def point_cloud_publish(self,pcd):
        pcl_pub = rospy.Publisher("augmented_pcl", PointCloud2)
        rospy.loginfo("Initializing sample pcl2 publisher node...")
        #header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'lidar'
        #create pcl from points
        self.scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, pcd.astype(np.float32))
        #rospy.loginfo(pcd.astype(np.float32))
        pcl_pub.publish(self.scaled_polygon_pcl)
        self.rate.sleep()
    
    def point_cloud_subscribe(self,data_1):
        self.point_background=data_1
        #xyz_array = ros_numpy.point_cloud2.get_xyz_points(self.point_background)
        xyz_array = pcl2.read_points_list(self.point_background)
        rospy.loginfo(xyz_array[0])

if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''

    with open('/home/santhanam.17/Carla_scripts/point_cloud_database.pickle', 'rb') as handle:
        read_dict = pickle.load(handle)
    try:
        x=Augmented_PCL_Publish(read_dict)
        
    except rospy.ROSInterruptException:
        pass
    
