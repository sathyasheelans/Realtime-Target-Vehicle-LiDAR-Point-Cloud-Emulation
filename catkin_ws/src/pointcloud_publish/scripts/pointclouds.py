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

class Augmented_PCL_Publish:

    def __init__(self,dict):
        rospy.init_node('augmented_pointcloud_pub', anonymous=True)
        self.rospy.Subscriber("target_pose",Float32MultiArray,self.callback)
        self.pcl_dict=dict
        self.rate=rospy.Rate(10)

    def callback(self,data):

        position_info=data.data

    def closest_point(self, arr):

        return arr
    
    def point_cloud_publish(self):

        rospy.init_node('pointcloud')
        
        while not rospy.is_shutdown():
            rospy.loginfo("happily publishing sample pointcloud.. !")
            l=rospy.Subscriber("position_info",Float32MultiArray,callback)
            print(position_info)

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
        x=Augmented_PCL_Publish(read_dict)
        x.point_cloud_publish()
    except rospy.ROSInterruptException:
        pass
    
