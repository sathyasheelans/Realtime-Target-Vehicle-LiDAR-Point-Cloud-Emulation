#!/usr/bin/env python
import rospy
import math
import sys
import numpy as np
import open3d as o3d

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''
    rospy.init_node('pcl2_pub_example')

    ## read the points from the pointcloud object
    pcd1 = o3d.io.read_point_cloud("/home/santhanam.17/Carla_scripts/processed/06:36:53sync.ply")
    point_cloud_in_numpy = np.asarray(pcd1.points)
    pcl_pub = rospy.Publisher("my_pcl_topic", PointCloud2)
    rospy.loginfo("Initializing sample pcl2 publisher node...")
    #give time to roscore to make the connections
    rospy.sleep(1.)
    #header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    #create pcl from points
    scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, point_cloud_in_numpy)
    #publish    
    
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo("happily publishing sample pointcloud.. !")
        pcl_pub.publish(scaled_polygon_pcl)
        rate.sleep()
