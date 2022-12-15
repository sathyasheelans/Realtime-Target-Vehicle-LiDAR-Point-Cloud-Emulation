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