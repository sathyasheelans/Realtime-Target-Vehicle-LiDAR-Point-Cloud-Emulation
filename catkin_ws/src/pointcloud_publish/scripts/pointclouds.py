#!/usr/bin/env python
import rospy
import math
import sys
import numpy as np
import open3d as o3d
import pickle

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
from std_msgs.msg import Float32MultiArray
import sensor_msgs.point_cloud2 as pcl2

global postion_info

def callback(data):
    position_info=data.data
    rospy.loginfo(data.data)

def point_cloud(read_dict):

    rospy.init_node('pointcloud')
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo("happily publishing sample pointcloud.. !")
        rospy.Subscriber("position_info",Float32MultiArray,callback)
        
        for item in read_dict.values():
            
            pcl_pub = rospy.Publisher("pointcloud_topic", PointCloud2)
            rospy.loginfo("Initializing sample pcl2 publisher node...")
            #header
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'
            #create pcl from points
            scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, item.astype(np.float32))
            pcl_pub.publish(scaled_polygon_pcl)
            rate.sleep()

if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''
    with open('/home/santhanam.17/Carla_scripts/filename.pickle', 'rb') as handle:
        read_dict = pickle.load(handle)
    try:
        point_cloud(read_dict)
    except rospy.ROSInterruptException:
        pass
    
