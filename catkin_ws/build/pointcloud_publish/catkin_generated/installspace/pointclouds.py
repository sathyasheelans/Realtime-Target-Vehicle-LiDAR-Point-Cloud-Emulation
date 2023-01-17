#!/usr/bin/env python3

"""
%% Software License Agreement (Proprietary)
%{
*********************************************************************
 *
 *  Copyright (c) 2022, The Ohio State University.
 *  All rights reserved.
 *  Created by:
 *            Sathyasheelan Santhanam, The Ohio State University
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution. 
 *   * Neither the name of Motional. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
%}

%{
%% Notes

Thi ROS node Subscribes to Target vehicle Position and Orientation and
Publishes the closest and Transformed pointcloud from the dataset 

%Written (and last modified) by Sathyasheelan Santhanam Jan 15, 2023.
%V2.0  Copyright The Ohio State University 2022.
%************************************************************
%}
"""
#import all dependancies

#Import all packages 
import rospy
import math
import numpy as np
import open3d as o3d
import pickle
from math import floor, ceil
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
import std_msgs.msg
from geometry_msgs.msg import Pose
import tf
#from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import sensor_msgs.point_cloud2 as pcl2


class Augmented_PCL_Publish:

    def __init__(self,dict):
        
        rospy.init_node('augmented_pointcloud_pub', anonymous=True) #Initialize the ros node "augmented_pointcloud_pub"
        self.rate=rospy.Rate(10) #Rate at wchih ROS messages are received 10 Hz

        #Initilize the variables
        self.pose=Pose()
        self.point_background=PointCloud2()
        self.Point_cloud=PointCloud()
        self.curr_position=[]
        self.pcl_dict=dict
        self.keys=list(self.pcl_dict.keys())
        self.last_callback_time = 0
        self.not_first_callback = False
        self.Transformation_Matrix= np.eye(4)
        
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("target", "point", rospy.Time(0),rospy.Duration(10))
        (self.trans,self.rot) = self.listener.lookupTransform('target', 'point', rospy.Time(0))

        self.pose_subscriber=rospy.Subscriber('target_pose',Pose,self.update_callback)
        #self.points_raw_subscriber=rospy.Subscriber('points_raw',PointCloud2,self.point_cloud_subscribe)

        rospy.spin()
        
    def update_callback(self,data):
        callback_time = rospy.get_time()
        if self.not_first_callback:
            time_since_last_callback = rospy.get_time() - self.last_callback_time
            rospy.loginfo(time_since_last_callback)

        self.pose=data
        self.curr_position=np.array([self.pose.position.x,self.pose.position.y,self.pose.position.z])
        self.orientation=self.pose.orientation.w
        self.point_cloud_publish(self.closest_point(self.curr_position,self.orientation))
        self.not_first_callback = True
        self.last_callback_time = callback_time
    
    def print_pose(self):
        rospy.loginfo(self.curr_position)

    def closest_point(self, arr, orr):
        Ego_postion=np.array([0,0,0])
        dist = round(np.linalg.norm(arr - Ego_postion))
        lst=list(range(-180,180,3))

        if dist>=120:
            dist=120
        elif dist<5:
            dist=5

        closet_yaw_rate=lst[min(range(len(lst)), key = lambda i: abs(lst[i]-orr))]
        closest_keys=[k for k in self.keys if k[4]==dist and k[3]==closet_yaw_rate ]
        pcd=self.pcl_dict[closest_keys[0]]

        yaw=math.radians(orr-closet_yaw_rate)
        self.Transformation_Matrix[0,0]=math.cos(yaw)
        self.Transformation_Matrix[0,1]=-math.sin(yaw)
        self.Transformation_Matrix[1,0]=math.sin(yaw)
        self.Transformation_Matrix[1,1]=math.cos(yaw)
        self.Transformation_Matrix[0,3]=arr[0]-closest_keys[0][0]
        self.Transformation_Matrix[1,3]=arr[1]-closest_keys[0][1]
        self.Transformation_Matrix[2,3]=arr[2]-closest_keys[0][2]

        return pcd

    def point_cloud_publish(self,pcd):

        #Function to call pointcloud transform
        Transformed_pcd=self.point_cloud_transform_o3d(pcd)

        #header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'point'

        #Pointcloud publishers
        transformed_pcl_pub=rospy.Publisher("transformed_pcl",PointCloud2,queue_size=10)

        #create pointcloud from points
        self.transformed_point_cloud = pcl2.create_cloud_xyz32(header, Transformed_pcd.astype(np.float32))
        transformed_pcl_pub.publish(self.transformed_point_cloud)
        self.rate.sleep()
    
    def point_cloud_subscribe(self,data_1):
        self.point_background=data_1
        #xyz_array = ros_numpy.point_cloud2.get_xyz_points(self.point_background)
        xyz_array = pcl2.read_points_list(self.point_background)
        rospy.loginfo(xyz_array[0])
    
    def point_cloud_transform_o3d(self,pcd):
        
        point_cloud_1 = o3d.geometry.PointCloud()
        point_cloud_1.points = o3d.utility.Vector3dVector(pcd)
        transformed_point_cloud=point_cloud_1.transform(self.Transformation_Matrix)
        transformed_pcd = np.asarray(transformed_point_cloud.points)

        return transformed_pcd
        

if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''

    with open('/home/santhanam.17/Carla_scripts/point_cloud_database_main_optimized.pickle', 'rb') as handle:
        read_dict = pickle.load(handle)
    try:
        x=Augmented_PCL_Publish(read_dict)
        
    except rospy.ROSInterruptException:
        pass
    
