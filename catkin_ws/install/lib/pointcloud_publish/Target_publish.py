#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from geometry_msgs.msg import Pose
import itertools

class Target_Trajectory:

    def __init__(self):
        rospy.init_node('target_position', anonymous=True)
        self.rate=rospy.Rate(10)
        self.pose=Pose()
        trajectory_array=[]
        trajectory_array=self.pos_target()
        pos_pub = rospy.Publisher("target_pose", Pose)
        rospy.loginfo("Initializing sample pos publisher node...")
        
        while not rospy.is_shutdown(): 
            for i in trajectory_array:
                self.pose.position.x=i[0]
                self.pose.position.y=i[1]
                self.pose.position.z=i[2]
                rospy.loginfo(self.pose)
                pos_pub.publish(self.pose)
            self.rate.sleep()


    def pos_target(self):
        trajectory=np.array([[-25,0,0],[-10,0,0],[-7,0,0],[-6,5,0],[7,5,0],[9,0,0],[25,0,0]])
        trajectory_extrapolated=[]
        for x in range(len(trajectory)-1):
            trajectory_extrapolated.append(np.linspace(trajectory[x],trajectory[x+1],30)[:-1])
        trajectory_extrapolated=np.array(list(itertools.chain.from_iterable(trajectory_extrapolated)))
        return trajectory_extrapolated


if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''
    try:
        X=Target_Trajectory()
    except rospy.ROSInterruptException:
        pass