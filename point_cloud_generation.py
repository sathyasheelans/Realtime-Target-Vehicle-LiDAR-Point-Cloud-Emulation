## Import Carla and other packages
import carla
import math
import random
import time
import numpy as np
import cv2
import open3d as o3d
from matplotlib import cm
import glob
import os
import sys
import argparse
from datetime import datetime
import random
import pickle

def process_lidar_sematic(point_cloud):
    # data.save_to_disk(parent_path + 'semantic-%06d.ply' % data.frame_number)
    parent_path='/home/santhanam.17/Carla_scripts/point/'

    #Extracting the raw data
    data = np.frombuffer(point_cloud.raw_data, dtype=np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('CosAngle', np.float32), ('ObjIdx', np.uint32), ('ObjTag', np.uint32)]))
    data_updated=data[data['ObjTag']==10]
    points_temp = np.array([data_updated['x'], data_updated['y'], data_updated['z']]).T
    point_list=o3d.geometry.PointCloud()
    point_list.points = o3d.utility.Vector3dVector(points_temp)
    o3d.io.write_point_cloud("/home/santhanam.17/Carla_scripts/point/"+str(point_cloud.frame)+".ply", point_list)

    #carla.LidarMeasurement.save_to_disk(points_new,"/home/santhanam.17/Carla_scripts/point/a.ply")
    #print(points)

def start_lidar():
    # --------------
    # Add a new LIDAR sensor to my ego
    lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
    lidar_bp.set_attribute('channels',str(64))
    lidar_bp.set_attribute('points_per_second',str(1280000))
    lidar_bp.set_attribute('rotation_frequency',str(10))
    lidar_bp.set_attribute('range',str(120))
    lidar_bp.set_attribute('upper_fov',str(0))
    lidar_bp.set_attribute('lower_fov',str(-30))
    lidar_location = carla.Location(0,0,2.5)
    lidar_rotation = carla.Rotation(0,0,0)
    lidar_transform = carla.Transform(lidar_location,lidar_rotation)
    lidar_sen = world.spawn_actor(lidar_bp,lidar_transform,attach_to=Ego_Vehicle)
    #Turn on Lidar and start listening
    lidar_sen.listen(lambda lidar_data: process_lidar_sematic(lidar_data))
    #lidar_sen.listen(lambda point_cloud: point_cloud.save_to_disk('/home/santhanam.17/Carla_scripts/point/%.6d.ply' % point_cloud.frame))
    time.sleep(1)
    #Destro the lidar
    lidar_sen.destroy()

def merge_ply():
    # assign directory
    directory = '/home/santhanam.17/Carla_scripts/point'
    pcds=np.empty([1,3])

    now = datetime.now()

    current_time = now.strftime("%H:%M:%S")

    for filename in os.listdir(directory):
        f = os.path.join(directory, filename)
        # checking if it is a file
        if os.path.isfile(f):
            print(f)
            pcd1 = o3d.io.read_point_cloud(f)
            point_cloud_in_numpy = np.asarray(pcd1.points)
            pcds =np.append(pcds,point_cloud_in_numpy,axis=0)
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(pcds)
            os.remove(f)

    #o3d.io.write_point_cloud("/home/santhanam.17/Carla_scripts/processed/"+str(current_time)+"_Final_Point_cloud.ply", point_cloud,write_ascii=True)
    #final_ply = o3d.io.read_point_cloud("/home/santhanam.17/Carla_scripts/processed/"+str(current_time)+"_sync.ply")
    #o3d.visualization.draw_geometries([final_ply])

    return point_cloud

def pointcloud_gen():
    #Create a loop to update the location of the target vehicle
    Target_Vehicle.set_transform(carla.Transform(carla.Location(x=5,y=0, z=0.598),carla.Rotation(pitch=0.0, yaw=0.0, roll=0.000000)))

    dict={}

    for i in range(5,120,2):
        print(i)
        for j in range(0,360,1):

            transform=Target_Vehicle.get_transform()
            transform.location.x = i*math.sin(math.radians(j))
            transform.location.y = i*math.cos(math.radians(j))
            Target_Vehicle.set_transform(transform)
            key_tuple=(transform.location.x,transform.location.y,transform.location.z\
                ,transform.rotation.pitch,transform.rotation.yaw,transform.rotation.roll)
            start_lidar()
            point_cloud=merge_ply()
            point_cloud_in_numpy = np.asarray(point_cloud.points)
            dict.update({key_tuple:point_cloud_in_numpy})
            time.sleep(0.1)

    with open('/home/santhanam.17/Carla_scripts/point_cloud_database.pickle', 'wb') as handle:
        pickle.dump(dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

pointcloud_gen()
