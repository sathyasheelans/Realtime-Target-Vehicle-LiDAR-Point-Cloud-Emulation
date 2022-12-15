#!/usr/bin/env python
import numpy as np
import itertools
from geometry_msgs.msg import Pose
class Target_Trajectory:
    

    def __init__(self):
        trajectory_array=[]
        trajectory_array=self.pos_target()
        self.pose=Pose()
        print(self.pose)
        for i in trajectory_array:
            print(i)
            self.pose.position.x=i[0]
            self.pose.position.y=i[1]
            self.pose.position.z=i[2]
            print(self.pose)
            



    def pos_target(self):
        trajectory=np.array([[-20,0,0],[-10,0,0],[-7,0,0],[-2,4,0],[7,4,0],[10,0,0],[20,0,0]])
        trajectory_extrapolated=[]
        for x in range(len(trajectory)-1):
            trajectory_extrapolated.append(np.linspace(trajectory[x],trajectory[x+1],10))
        trajectory_extrapolated=np.array(list(itertools.chain.from_iterable(trajectory_extrapolated)))
        return trajectory_extrapolated

if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''
    try:
        X=Target_Trajectory()
    except:
        pass


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
