#!/usr/bin/env python
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

        for j in range(0,360,1):

            transform=Target_Vehicle.get_transform()
            transform.location.x = i*math.sin(math.radians(j))
            transform.location.y = i*math.cos(math.radians(j))
            Target_Vehicle.set_transform(transform)
            key_tuple=(transform.location.x,transform.location.y,transform.location.z\
                ,transform.rotation.pitch,transform.rotation.yaw,transform.rotation.roll)
            start_lidar()
            point_cloud1=merge_ply()
            point_cloud_in_numpy = np.asarray(point_cloud1.points)
            dict.update({key_tuple:point_cloud_in_numpy})



    with open('/home/santhanam.17/Carla_scripts/point_cloud_database.pickle', 'wb') as handle:
        pickle.dump(dict, handle, protocol=pickle.HIGHEST_PROTOCOL)


if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''
    # Connect the client and set up bp library and spawn point
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    bp_lib = world.get_blueprint_library() 
    spawn_points = world.get_map().get_spawn_points() 
    world.unload_map_layer(carla.MapLayer.Foliage)
    world.unload_map_layer(carla.MapLayer.Buildings)
    world.unload_map_layer(carla.MapLayer.Decals)
    world.unload_map_layer(carla.MapLayer.ParkedVehicles)
    world.unload_map_layer(carla.MapLayer.Props)
    world.unload_map_layer(carla.MapLayer.StreetLights)
    world.unload_map_layer(carla.MapLayer.Walls)
    world.unload_map_layer(carla.MapLayer.Particles)
    world.load_map_layer(carla.MapLayer.Ground)
    ##Selecting a spawn point for the ego vehicle
    spawnPoint=carla.Transform(carla.Location(x=0,y=0, z=0.598),carla.Rotation(pitch=0.0, yaw=0.0, roll=0.000000))
    #Spawn Ego vehicle
    Ego_Vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020') 
    Ego_Vehicle = world.try_spawn_actor(Ego_Vehicle_bp, spawnPoint)

    # Move spectator to view ego vehicle
    spectator = world.get_spectator() 
    transform = carla.Transform(Ego_Vehicle.get_transform().transform(carla.Location(x=-25,z=10,y=0)),Ego_Vehicle.get_transform().rotation) 
    spectator.set_transform(transform)
    ##Selecting a random spawn point for the target vehicle
    spawnPoint1=carla.Transform(carla.Location(x=10,y=0, z=0.598),carla.Rotation(pitch=0.0, yaw=0.0, roll=0.000000))
    #Spawn Target vehicle
    Target_Vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020') 
    Target_Vehicle = world.try_spawn_actor(Target_Vehicle_bp, spawnPoint1)
    pointcloud_gen()