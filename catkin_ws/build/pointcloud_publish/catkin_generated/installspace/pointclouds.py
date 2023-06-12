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
 *   * Neither the name of The Ohio State University. nor the names of its
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

This ROS node Subscribes to Target vehicle Position and Orientation and
Publishes the closest and Transformed pointcloud from the dataset.

This ROS Node will also subscribe to real-time environment pointcloud and 
merge with the POVs transformed pointcloud and publish them at 10 Hz

%Written (and last modified) by Sathyasheelan Santhanam Feb 06, 2023.
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
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import PointCloud
import std_msgs.msg
from geometry_msgs.msg import Pose
import tf
import ros_numpy
from collections import OrderedDict
from datetime import datetime
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import message_filters
from scipy.spatial import cKDTree
#from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import sensor_msgs.point_cloud2 as pcl2


class Augmented_PCL_Publish:

    def __init__(self,dict):
        
        rospy.init_node('augmented_pointcloud_pub', anonymous=True) #Initialize the ros node "augmented_pointcloud_pub"
        self.rate=rospy.Rate(10) #Rate at wchih ROS messages are received 10 Hz

        #Initilize the variables
        self.pose=Pose()
        self.point_background=PointCloud2()
        #self.Point_cloud=PointCloud()
        self.curr_position=[]
        self.pcl_dict=dict
        self.keys=list(self.pcl_dict.keys())
        self.last_callback_time = 0
        self.not_first_callback = False
        self.Transformation_Matrix= np.eye(4)
        
        #self.listener = tf.TransformListener()
        #self.listener.waitForTransform("target", "point", rospy.Time(0),rospy.Duration(10))
        #(self.trans,self.rot) = self.listener.lookupTransform('target', 'point', rospy.Time(0))

        #Initialize a subscriber to subscribe to background to real-time background pointcloud
        #self.points_raw_subscriber=rospy.Subscriber('points_raw',PointCloud2,self.point_cloud_subscribe)
        self.points_raw_subscriber=rospy.Subscriber('points_raw',PointCloud2,self.point_cloud_subscribe)

        #Initialize a Subscriber to subscribe to the pose (postion and orientation) of the target vehicle
        self.pose_subscriber=rospy.Subscriber('target_pose',Pose,self.update_callback)

        #self.points_raw=message_filters.Subscriber('points_raw',PointCloud2)
        #self.pose_sub=message_filters.Subscriber('target_pose',PoseStamped)

        #ts = message_filters.TimeSynchronizer([self.points_raw, self.pose_sub], 10)
        #ts.registerCallback(self.ucallback)
        
        rospy.spin()
        #self.rate.sleep()

    def ucallback(self, data_1, data):
        
        rospy.loginfo("hi")

        self.point_background=data_1
        #xyz_array = ros_numpy.point_cloud2.get_xyz_points(self.point_background)
        #xyz_array = pcl2.read_points_list(self.point_background, skip_nans=True, field_names=("x", "y", "z"))
        real_pointcloud_num = ros_numpy.numpify(data_1)
        self.real_pointcloud=np.zeros((real_pointcloud_num.shape[0],3))
        self.real_pointcloud[:,0]=real_pointcloud_num['x']
        self.real_pointcloud[:,1]=real_pointcloud_num['y']
        self.real_pointcloud[:,2]=real_pointcloud_num['z']

        #check the time taken to perform the code 
        callback_time = rospy.get_time()
        if self.not_first_callback:
            time_since_last_callback = rospy.get_time() - self.last_callback_time
            rospy.loginfo(time_since_last_callback)

        #assign the pose values to the veriable
        self.pose=data
        self.curr_position=np.array([self.pose.position.x,self.pose.position.y,self.pose.position.z])
        self.orientation=self.pose.orientation.w

        #Call the function which publishes the augmented point cloud
        self.point_cloud_publish(self.closest_point(self.curr_position,self.orientation))
        self.not_first_callback = True
        self.last_callback_time = callback_time

    
    #Callback function of "target_pose" subscriber
    def update_callback(self,data):
        #rospy.loginfo(data.header.stamp)
        #check the time taken to perform the code 
        
        callback_time = rospy.get_time()
        if self.not_first_callback:
            time_since_last_callback = rospy.get_time() - self.last_callback_time
            #rospy.loginfo(time_since_last_callback)

        #assign the pose values to the veriable
        self.pose=data
        self.curr_position=np.array([self.pose.position.x,self.pose.position.y,self.pose.position.z])
        self.orientation=self.pose.orientation.w

        #Call the function which publishes the augmented point cloud
        self.point_cloud_publish(self.closest_point(self.curr_position,self.orientation))
        self.not_first_callback = True
        self.last_callback_time = callback_time

        duration = rospy.get_time() - callback_time
        rospy.loginfo("update_callback %s",duration)

        #rospy.spin()
    
    #Callback function of "point_raw" subscriber
    def point_cloud_subscribe(self,data_1):
        #rospy.loginfo(data_1.header.stamp)
        #rospy.loginfo(rospy.get_time())
        self.point_background=data_1
        #xyz_array = ros_numpy.point_cloud2.get_xyz_points(self.point_background)
        #xyz_array = pcl2.read_points_list(self.point_background, skip_nans=True, field_names=("x", "y", "z"))
        real_pointcloud_num = ros_numpy.numpify(data_1)
        self.real_pointcloud=np.zeros((real_pointcloud_num.shape[0],4))
        self.real_pointcloud[:,0]=real_pointcloud_num['x']
        self.real_pointcloud[:,1]=real_pointcloud_num['y']
        self.real_pointcloud[:,2]=real_pointcloud_num['z']
        self.real_pointcloud[:,3]=real_pointcloud_num['intensity']
        #rospy.loginfo("check")
        #rospy.loginfo(real_pointcloud_num['intensity'])

        

    #function to find the closest pointclound orientation based on target vehicle location and heading angle
    def closest_point(self, arr, orr):

        start_time = rospy.get_time()
        Ego_postion=np.array([0,0,2])
        dist = round(np.linalg.norm(arr - Ego_postion))
        lst=list(range(0,360,1))

        # Calculate the rotation angle with respect to the ego vehicle
        yaw_angle=orr #heading angle
        #if arr[0] and arr[1] >0:
        normal_angle=math.degrees(math.atan(abs(arr[0]/arr[1]))) #angle between the ego vehicle and target vehicle normals
        #else:
            #normal_angle=math.degrees(math.atan(abs(arr[0]/arr[1])))
        
        closest_orientation=0
        if arr[1]<0 and arr[0]>=0: #1st Quadrant
            closest_orientation=90+yaw_angle-normal_angle
        elif arr[1]<=0 and arr[0]<0: #4th Quadrant
            closest_orientation=90+yaw_angle+normal_angle
        elif arr[1]>0 and arr[0]<=0: #3rd Quadrant
            closest_orientation=270+yaw_angle-normal_angle
        elif arr[1]>0 and arr[0]>=0: #2nd Quadrant
            closest_orientation=270+yaw_angle+normal_angle

        if closest_orientation>360:
            closest_orientation=closest_orientation%360

            #closest_orientation=-closest_orientation
        if dist>=120:
            dist=120
        elif dist<5:
            dist=5
        """rospy.loginfo(arr[0])
        rospy.loginfo(arr[1])
        rospy.loginfo(closest_orientation)
        rospy.loginfo(yaw_angle)"""
        #closest orientation from the list of orientation
        closet_yaw_rate=lst[min(range(len(lst)), key = lambda i: abs(lst[i]-closest_orientation))]
        closest_keys=[k for k in self.keys if k[4]==dist and k[3]==closet_yaw_rate ] #get the key of that pointcloud
        pcd=self.pcl_dict[closest_keys[0]] #get the pointcloud
        #rospy.loginfo(dist)
        #rospy.loginfo(len(pcd))
        """rospy.loginfo(closest_keys[0][0])
        rospy.loginfo(closest_keys[0][1])
        rospy.loginfo(closest_keys[0][2])"""
        #rospy.loginfo(closet_yaw_rate)
        
        #rotation angle along Z axis
        rotation_angle=math.radians(orr-closest_orientation)

        #Transformation matrix 
        self.Transformation_Matrix[0,0]=math.cos(rotation_angle)
        self.Transformation_Matrix[0,1]=-math.sin(rotation_angle)
        self.Transformation_Matrix[1,0]=math.sin(rotation_angle)
        self.Transformation_Matrix[1,1]=math.cos(rotation_angle)
        self.Transformation_Matrix[0,3]=arr[0]-closest_keys[0][0]
        self.Transformation_Matrix[1,3]=arr[1]-closest_keys[0][1]
        self.Transformation_Matrix[2,3]=arr[2]-closest_keys[0][2]

        duration = rospy.get_time() - start_time
        rospy.loginfo("closest_point %s",duration)
        return pcd

    
    def point_cloud_publish(self,pcd):

              
        #Function to call pointcloud transform
        #Transformed_pcd, Orientedbounding_box , Axisalignedbunding_box=self.point_cloud_transform_o3d(pcd)
        #Transformed_pcd =  np.concatenate((Transformed_pcd_temp, pcd[:,3].reshape(-1,1)), axis=1)
        #rospy.loginfo(np.shape(Transformed_pcd))
        #rospy.loginfo(np.shape(pcd[:,:3]))
        #rospy.loginfo(np.shape(pcd[:,3].reshape(-1,1)))

        Transformed_pcd_temp, Orientedbounding_box , Axisalignedbunding_box=self.point_cloud_transform_o3d(pcd)
        new_column = np.full((Transformed_pcd_temp.shape[0], 1), max(self.real_pointcloud[:,3]))
        Transformed_pcd =  np.concatenate((Transformed_pcd_temp,new_column), axis=1)
        
        #remove the background pointcloud
        A=self.real_pointcloud
        point_cloud_background = o3d.geometry.PointCloud()
        #point_cloud_background.points = o3d.utility.Vector3dVector(self.real_pointcloud[:,:3])
        point_cloud_background.points = o3d.utility.Vector3dVector(self.real_pointcloud[:,:3])

        box_points=Axisalignedbunding_box.get_box_points()
        box_center=Axisalignedbunding_box.get_center()

        #remove the points inside the vehicle bounding box
        inliers_indices = Orientedbounding_box.get_point_indices_within_bounding_box(point_cloud_background.points)
        outliers_pcd = point_cloud_background.select_by_index(inliers_indices, invert=True)

        

        #visualization of the bounding box
        #self.visualization_boundingbox(np.array(Orientedbounding_box.get_box_points()))

        #Generate frustum base
        frustrum_points=self.frustrum_generation(box_points,box_center)
        frustrum_points_np=np.array(frustrum_points)
        
        #function to remove the points inside frustum
        
        #outliers_pcd_1=self.remove_points_inside_frustum(frustrum_points_np, np.array(point_cloud_background.points))
        outliers_pcd_1=self.remove_points_inside_frustum(frustrum_points_np, np.array(A))

        start_time = rospy.get_time()  
        #augment the transformed pcd with real-time pcd(removed)
        #augmented_pcd_1=np.concatenate((Transformed_pcd,np.array(outliers_pcd.points)))
        #B=np.array(outliers_pcd_1)
        
        """ 
       # Check which column in A has the same values as B
        matching_column = np.isin(A[:, :3], B).all(axis=1)

        #rospy.loginfo(np.shape(matching_column))
        # Extract the corresponding fourth column from A
        matching_values = A[matching_column][:, 3]

        # Append the matching values to B
        real_pointcloud_with_intensity = np.concatenate((B, matching_values[:, None].reshape(-1,1)), axis=1)
        """

        """A_values = A[:, :3]
        A_intensities = A[:, 3]

        tree = cKDTree(A_values)

        distances, indices = tree.query(B, k=1)

        corresponding_intensities = np.where(distances == 0, A_intensities[indices], 0)

        C = np.column_stack((B, corresponding_intensities))"""

        augmented_pcd=np.concatenate((Transformed_pcd,outliers_pcd_1))
        #augmented_pcd=np.concatenate((Transformed_pcd,C))

        #header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'lidar'

        #Pointcloud publishers
        #transformed_pcl_pub=rospy.Publisher("transformed_pcl",PointCloud2,queue_size=10)
        augmented_pcl_pub=rospy.Publisher("augmented_pcl",PointCloud2,queue_size=10)

        
        
        #create pointcloud from points
        #self.transformed_point_cloud = pcl2.create_cloud_xyz32(header, augmented_pcd_1.astype(np.float32))
        #self.augmented_point_cloud = pcl2.create_cloud_xyz32(header, augmented_pcd.astype(np.float32))
        self.augmented_point_cloud=self.pointcloud2_creation(augmented_pcd)

        

        #transformed_pcl_pub.publish(self.transformed_point_cloud)
        augmented_pcl_pub.publish(self.augmented_point_cloud)

        duration = rospy.get_time() - start_time
        rospy.loginfo("point_cloud_publish %s",duration)
        #self.rate.sleep()
    
    #Function to crerate a PointCloud2 msg

    def pointcloud2_creation(self, pcd):

        numpy_array=pcd.astype(np.float32)
        # Create the PointCloud2 message
        cloud_msg = PointCloud2()

        # Set the fields of the PointCloud2 message
        cloud_msg.header.stamp = rospy.Time.now()
        cloud_msg.header.frame_id = 'lidar'
        cloud_msg.height = 1
        cloud_msg.width = len(numpy_array)
        cloud_msg.fields.append(PointField('x', 0, PointField.FLOAT32, 1))
        cloud_msg.fields.append(PointField('y', 4, PointField.FLOAT32, 1))
        cloud_msg.fields.append(PointField('z', 8, PointField.FLOAT32, 1))
        cloud_msg.fields.append(PointField('intensity', 12, PointField.FLOAT32, 1))
        cloud_msg.point_step = 16  # Size of each point in bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_bigendian = False
        cloud_msg.is_dense = True

        # Create a structured array from the numpy_array and intensity_array
        structured_array = np.zeros(len(numpy_array), dtype=[('x', np.float32), ('y', np.float32),
                                                            ('z', np.float32), ('intensity', np.float32)])
        structured_array['x'] = numpy_array[:, 0]
        structured_array['y'] = numpy_array[:, 1]
        structured_array['z'] = numpy_array[:, 2]
        structured_array['intensity'] = numpy_array[:, 3]
        #structured_array['intensity'] = 1.0

        # Flatten the structured array and assign it as the point cloud data
        cloud_msg.data = structured_array.tobytes()

        return cloud_msg

    #Function to get the bounding box and perform transformation of the pointcloud and return the transformed pcl
    def point_cloud_transform_o3d(self,pcd):
        
        start_time = rospy.get_time()
        point_cloud_1 = o3d.geometry.PointCloud()
        pcd = [x for x in pcd if x[0]!=0 and x[1]!=0]
        pcd = np.unique(pcd, axis =0)
        #point_cloud_1.points = o3d.utility.Vector3dVector(pcd[:,:3])
        point_cloud_1.points = o3d.utility.Vector3dVector(pcd)
        transformed_point_cloud=point_cloud_1.transform(self.Transformation_Matrix)
        boundingbox_POV=o3d.geometry.OrientedBoundingBox.create_from_points(transformed_point_cloud.points)
        boundingbox_POV_axis=o3d.geometry.AxisAlignedBoundingBox.create_from_points(transformed_point_cloud.points)
        transformed_pcd = np.asarray(transformed_point_cloud.points)
        #Transformed_pcd =  np.concatenate((transformed_pcd, pcd[:,3].reshape(-1,1)), axis=1)

        duration = rospy.get_time() - start_time
        rospy.loginfo("point_cloud_transform_o3d %s",duration)
        return transformed_pcd, boundingbox_POV, boundingbox_POV_axis

    #Function to get the generate frustum corners based on Bounding box position and orientation   
    def frustrum_generation(self,box_points,target_location):

        start_time = rospy.get_time()
        lidar_location=[0,0,0]
        #self.visualization_Lidar(lidar_location)

        #Distance between lidar and target vehicle
        Lidar_distance = round(np.linalg.norm(np.array(lidar_location)-np.array(target_location)))

        #normal vector 
        x_norm=target_location[0]-lidar_location[0]
        y_norm=target_location[1]-lidar_location[1]
        z_norm=target_location[2]-lidar_location[2]

        #equation of the plane ax+by+cz=d
        d=(x_norm*target_location[0]+y_norm*target_location[1]+z_norm*target_location[2])
        plane=[x_norm,y_norm,z_norm,d] #a,b,c,d

        #projection of the box points on to the normal plane
        points_projected=[]
        tuple_array=[]

        for point in box_points:

            k =(plane[3] - (plane[0]*point[0]) - (plane[1]*point[1]) - (plane[2]*point[2]))/(plane[0]*plane[0] + plane[1]*plane[1] +plane[2]*plane[2])
            proj_point=[point[0]+(k*plane[0]), point[1]+(k*plane[1]), point[2]+(k*plane[2])]
            points_projected.append(proj_point)
            dist = np.linalg.norm(np.array(proj_point)-np.array(target_location))
            #rospy.loginfo(dist)
            #direction vector between the projected points to get unique projection
            dv=[lidar_location[0]-proj_point[0],lidar_location[1]-proj_point[1],lidar_location[2]-proj_point[2]]
            #create a object tuple
            object_tuple=tuple([dist,tuple(proj_point),tuple(dv)])
            if object_tuple not in tuple_array:
                tuple_array.append(object_tuple)

        #sort teh tuple array based on distance
        tuple_array.sort(key=lambda x:x[0])
        points_projected_sorted=[list(x[1]) for x in tuple_array]

        #self.visualization_projected(points_projected_sorted)

        if len(points_projected_sorted)>4:
            points_projected_sorted=points_projected_sorted[4:]

        
        #Generation of Frustrum points
        frustrum_points=[]

        #Sort the projected corners to determine TOP left, TOP right, BOTTOM left, BOTTOM right
        target_location_max_z=[target_location[0],target_location[1],1]
        k_1 =(plane[3] - (plane[0]*target_location_max_z[0]) - (plane[1]*target_location_max_z[1]) - (plane[2]*target_location_max_z[2]))/(plane[0]*plane[0] + plane[1]*plane[1] +plane[2]*plane[2])
        extended_point=[target_location_max_z[0]+(k_1*plane[0]), target_location_max_z[1]+(k_1*plane[1]), target_location_max_z[2]+(k_1*plane[2])]

        tl=[]
        tr=[]
        bl=[]
        br=[]
        line_start = np.array(target_location)
        line_end = np.array(extended_point)
        # Calculate the vectors formed by the line and the point
        line_vector = line_end - line_start 

        #rospy.loginfo(points_projected_sorted)
        for point in points_projected_sorted:
            # Convert points to numpy arrays
            point = np.array(point)
            point_vector = point - line_start
            # Calculate the cross product of the vectors
            cross_product = np.cross(line_vector, point_vector)
            #rospy.loginfo(cross_product)
            #rospy.loginfo(point[2])
            #rospy.loginfo(target_location[2])
            if point[2]>target_location[2]:
                if cross_product[2]>0:
                    tl=point
                    #rospy.loginfo("tl")
                else:
                    tr=point
                    #rospy.loginfo("tr")
            else:
                if cross_product[2]<0:
                    br=point
                    #rospy.loginfo("br")
                else:
                    bl=point
                    #rospy.loginfo("bl")

        points_projected_sorted=[tl,tr,br,bl]
        #rospy.loginfo(points_projected_sorted)

        
        #self.visualization_base(points_projected_sorted)

        #Generate far corners of the frustum
        for p in list(points_projected_sorted):

            vx=(p[0]-lidar_location[0])
            vy=(p[1]-lidar_location[1])
            vz=(p[2]-lidar_location[2])


            length=math.sqrt((vx*vx)+(vy*vy)+(vz*vz))
            vx_norm=100*vx/length
            vy_norm=100*vy/length
            vz_norm=100*vz/length

            frus_point=[p[0]+vx_norm,p[1]+vy_norm,p[2]+vz_norm]
            frustrum_points.append(frus_point)
            frustrum_points.append(p)

        #Visualization of the frustum
        #self.visualization_projection(frustrum_points)
        """top_left=[frustrum_points[0],frustrum_points[1]]
        top_right=[frustrum_points[2],frustrum_points[3]]
        bottom_left=[frustrum_points[4],frustrum_points[5]]
        bottom_right=[frustrum_points[6],frustrum_points[7]]"""
        #self.visualization_corner(bottom_right)

        duration = rospy.get_time() - start_time
        rospy.loginfo("frustrum_generation %s",duration)
        return frustrum_points

    #Function to perform frustum culling
    def remove_points_inside_frustum(self, frustum_corners, point_cloud):

        start_time = rospy.get_time()
        # Define the frustum using its corner points
        tln=frustum_corners[1]
        trn=frustum_corners[3]
        brn=frustum_corners[5]
        bln=frustum_corners[7]
        tlf=frustum_corners[0]
        trf=frustum_corners[2]
        brf=frustum_corners[4]
        blf=frustum_corners[6]

        # Calculate the frustum planes
        #front plane
        front_plane = np.cross(tln - brn, trn - brn)
        front_plane /= np.linalg.norm(front_plane)
        front_plane = np.concatenate((np.transpose(front_plane), -np.dot(front_plane, brn)), axis=None)

        #top plane
        top_plane = np.cross(tlf - tln, trf - tln)
        top_plane /= np.linalg.norm(top_plane)
        top_plane = np.concatenate((top_plane, -np.dot(top_plane, tln)), axis=None)
        
        #bottom plane
        bottom_plane = np.cross(brf - bln, blf - bln)
        bottom_plane /= np.linalg.norm(bottom_plane)
        bottom_plane = np.concatenate((bottom_plane, -np.dot(bottom_plane, bln)), axis=None )

        #far plane
        far_plane = np.cross(trf - blf, tlf - blf)
        far_plane /= np.linalg.norm(far_plane)
        far_plane = np.concatenate((far_plane, -np.dot(far_plane, blf)), axis=None)

        #left plane
        left_plane = np.cross(blf - tln, tlf - tln)
        left_plane /= np.linalg.norm(left_plane)
        left_plane = np.concatenate((left_plane, -np.dot(left_plane, tln)), axis=None)

        #right plane
        right_plane = np.cross(trf - trn, brf - trn)
        right_plane /= np.linalg.norm(right_plane)
        right_plane = np.concatenate((right_plane, -np.dot(right_plane, trn)), axis=None)

        planes = np.array([top_plane, bottom_plane, far_plane, front_plane, left_plane, right_plane])

        # Iterate through the point cloud and remove points inside the frustum
        inside_mask = np.ones(len(point_cloud), dtype=bool)
        for plane in planes:
            distances = np.dot(point_cloud[:,:3], plane[:3]) + plane[3]
            inside_mask = np.logical_and(inside_mask, distances > 0)
        point_cloud = point_cloud[~inside_mask]
        #rospy.loginfo(inside_frustum)

        duration = rospy.get_time() - start_time
        rospy.loginfo("remove_points_inside_frustum %s",duration)
        return point_cloud


    def visualization_base(self,box_points):
        # Create a marker publisher
        marker_pub = rospy.Publisher('visualization_marker_base', Marker, queue_size=10)

        # Define the marker message
        marker_msg = Marker()
        marker_msg.header.frame_id = 'lidar'
        marker_msg.type = Marker.LINE_STRIP 
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.3
        marker_msg.scale.y = 0.3
        marker_msg.scale.z = 0.3
        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0

        # Define the points that define the plane
        
        p1 = Point()
        p1.x = box_points[0][0]
        p1.y = box_points[0][1]
        p1.z = box_points[0][2]

        p2 = Point()
        p2.x = box_points[1][0]
        p2.y = box_points[1][1]
        p2.z = box_points[1][2]

        p3 = Point()
        p3.x = box_points[2][0]
        p3.y = box_points[2][1]
        p3.z = box_points[2][2]

        p4 = Point()
        p4.x = box_points[3][0]
        p4.y = box_points[3][1]
        p4.z = box_points[3][2]

    
        # Set the points field of the marker message
        marker_msg.points = [p1, p2, p3, p4, p1]
        marker_pub.publish(marker_msg)

    def visualization_corner(self,box_points):
        # Create a marker publisher
        marker_pub = rospy.Publisher('visualization_marker_corner', Marker, queue_size=10)

        # Define the marker message
        marker_msg = Marker()
        marker_msg.header.frame_id = 'lidar'
        marker_msg.type = Marker.POINTS 
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.3
        marker_msg.scale.y = 0.3
        marker_msg.scale.z = 0.3
        marker_msg.color.a = 1.0
        marker_msg.color.r = 0.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 1.0

        # Define the points that define the plane
        
        p1 = Point()
        p1.x = box_points[0][0]
        p1.y = box_points[0][1]
        p1.z = box_points[0][2]

        p2 = Point()
        p2.x = box_points[1][0]
        p2.y = box_points[1][1]
        p2.z = box_points[1][2]


    
        # Set the points field of the marker message
        marker_msg.points = [p1, p2]
        marker_pub.publish(marker_msg)

    def visualization_Lidar(self,location):
        # Create a marker publisher
        marker_pub = rospy.Publisher('visualization_marker_lidar', Marker, queue_size=10)

        # Define the marker message
        marker_msg = Marker()
        marker_msg.header.frame_id = 'lidar'
        marker_msg.type = Marker.CYLINDER 
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 1
        marker_msg.scale.y = 1
        marker_msg.scale.z = 1
        marker_msg.color.a = 1.0
        marker_msg.color.r = 0.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 1.0

        # Define the points that define the plane
        
        p1 = Point()
        p1.x = location[0]
        p1.y = location[1]
        p1.z = location[2]
    
        # Set the points field of the marker message
        marker_msg.points = [p1]
        marker_pub.publish(marker_msg)

    def visualization_far(self,box_points):
        # Create a marker publisher
        marker_pub = rospy.Publisher('visualization_marker_far', Marker, queue_size=10)

        # Define the marker message
        marker_msg = Marker()
        marker_msg.header.frame_id = 'lidar'
        marker_msg.type = Marker.LINE_STRIP 
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.3
        marker_msg.scale.y = 0.3
        marker_msg.scale.z = 0.3
        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0

        # Define the points that define the plane
        
        p1 = Point()
        p1.x = box_points[0][0]
        p1.y = box_points[0][1]
        p1.z = box_points[0][2]

        p2 = Point()
        p2.x = box_points[1][0]
        p2.y = box_points[1][1]
        p2.z = box_points[1][2]

        p3 = Point()
        p3.x = box_points[2][0]
        p3.y = box_points[2][1]
        p3.z = box_points[2][2]

        p4 = Point()
        p4.x = box_points[3][0]
        p4.y = box_points[3][1]
        p4.z = box_points[3][2]

    
        # Set the points field of the marker message
        marker_msg.points = [p1, p2, p3, p4, p1]
        marker_pub.publish(marker_msg)

    def visualization_projection(self,box_points):
        # Create a marker publisher
        marker_pub = rospy.Publisher('visualization_marker_projection', Marker, queue_size=10)

        # Define the marker message
        marker_msg = Marker()
        marker_msg.header.frame_id = 'lidar'
        marker_msg.type = Marker.LINE_STRIP 
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.3
        marker_msg.scale.y = 0.3
        marker_msg.scale.z = 0.3
        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0

        # Define the points that define the plane
        
        p1 = Point()
        p1.x = box_points[0][0]
        p1.y = box_points[0][1]
        p1.z = box_points[0][2]

        p2 = Point()
        p2.x = box_points[1][0]
        p2.y = box_points[1][1]
        p2.z = box_points[1][2]

        p3 = Point()
        p3.x = box_points[2][0]
        p3.y = box_points[2][1]
        p3.z = box_points[2][2]

        p4 = Point()
        p4.x = box_points[3][0]
        p4.y = box_points[3][1]
        p4.z = box_points[3][2]

    
        p5 = Point()
        p5.x = box_points[4][0]
        p5.y = box_points[4][1]
        p5.z = box_points[4][2]

        p6 = Point()
        p6.x = box_points[5][0]
        p6.y = box_points[5][1]
        p6.z = box_points[5][2]

        p7 = Point()
        p7.x = box_points[6][0]
        p7.y = box_points[6][1]
        p7.z = box_points[6][2]

        p8 = Point()
        p8.x = box_points[7][0]
        p8.y = box_points[7][1]
        p8.z = box_points[7][2]

        # Set the points field of the marker message
        marker_msg.points = [p1, p3, p5, p7, p1, p2, p4, p6, p8, p2, p4, p3, p5,p6,p8, p7 ]
        marker_pub.publish(marker_msg)

    def visualization_boundingbox(self,box_points):
        # Create a marker publisher
        marker_pub = rospy.Publisher('visualization_marker_boundingbox', Marker, queue_size=10)

        # Define the marker message
        marker_msg = Marker()
        marker_msg.header.frame_id = 'lidar'
        marker_msg.type = Marker.POINTS 
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.3
        marker_msg.scale.y = 0.3
        marker_msg.scale.z = 0.3
        marker_msg.color.a = 1.0
        marker_msg.color.r = 0.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0

        # Define the points that define the plane
        
        p1 = Point()
        p1.x = box_points[0][0]
        p1.y = box_points[0][1]
        p1.z = box_points[0][2]

        p2 = Point()
        p2.x = box_points[1][0]
        p2.y = box_points[1][1]
        p2.z = box_points[1][2]

        p3 = Point()
        p3.x = box_points[2][0]
        p3.y = box_points[2][1]
        p3.z = box_points[2][2]

        p4 = Point()
        p4.x = box_points[3][0]
        p4.y = box_points[3][1]
        p4.z = box_points[3][2]

    
        p5 = Point()
        p5.x = box_points[4][0]
        p5.y = box_points[4][1]
        p5.z = box_points[4][2]

        p6 = Point()
        p6.x = box_points[5][0]
        p6.y = box_points[5][1]
        p6.z = box_points[5][2]

        p7 = Point()
        p7.x = box_points[6][0]
        p7.y = box_points[6][1]
        p7.z = box_points[6][2]

        p8 = Point()
        p8.x = box_points[7][0]
        p8.y = box_points[7][1]
        p8.z = box_points[7][2]

        # Set the points field of the marker message
        marker_msg.points = [p1, p2,p3,p4,p5,p6,p7,p8 ]
        marker_pub.publish(marker_msg)
    def visualization_projected(self,box_points):
        # Create a marker publisher
        marker_pub = rospy.Publisher('visualization_marker_projected', Marker, queue_size=10)

        # Define the marker message
        marker_msg = Marker()
        marker_msg.header.frame_id = 'lidar'
        marker_msg.type = Marker.POINTS 
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.3
        marker_msg.scale.y = 0.3
        marker_msg.scale.z = 0.3
        marker_msg.color.a = 1.0
        marker_msg.color.r = 0.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 1.0

        # Define the points that define the plane
        
        p1 = Point()
        p1.x = box_points[0][0]
        p1.y = box_points[0][1]
        p1.z = box_points[0][2]

        p2 = Point()
        p2.x = box_points[1][0]
        p2.y = box_points[1][1]
        p2.z = box_points[1][2]

        p3 = Point()
        p3.x = box_points[2][0]
        p3.y = box_points[2][1]
        p3.z = box_points[2][2]

        p4 = Point()
        p4.x = box_points[3][0]
        p4.y = box_points[3][1]
        p4.z = box_points[3][2]

    
        p5 = Point()
        p5.x = box_points[4][0]
        p5.y = box_points[4][1]
        p5.z = box_points[4][2]

        p6 = Point()
        p6.x = box_points[5][0]
        p6.y = box_points[5][1]
        p6.z = box_points[5][2]

        p7 = Point()
        p7.x = box_points[6][0]
        p7.y = box_points[6][1]
        p7.z = box_points[6][2]

        p8 = Point()
        p8.x = box_points[7][0]
        p8.y = box_points[7][1]
        p8.z = box_points[7][2]

        # Set the points field of the marker message
        marker_msg.points = [p1, p2,p3,p4,p5,p6,p7,p8 ]
        marker_pub.publish(marker_msg)

if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''

    with open('/home/santhanam.17/Carla_scripts/point_cloud_database_main_optimized_2.pickle', 'rb') as handle:
    #with open('/home/santhanam.17/Carla_scripts/point_cloud_database_main_optimized_intensity.pickle', 'rb') as handle:
        
        read_dict = pickle.load(handle)
    try:
        x=Augmented_PCL_Publish(read_dict)
        
    except rospy.ROSInterruptException:
        pass
    
