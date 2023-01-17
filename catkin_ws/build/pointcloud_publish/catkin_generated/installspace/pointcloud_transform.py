#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from geometry_msgs.msg import Pose
import itertools
import math

class Transformation:

    def __init__(self):
        rospy.init_node('target_position1', anonymous=True)
        self.rate=rospy.Rate(10)
        self.pose=Pose()
        trajectory_array=[]
        yaw_rate=[]
        trajectory_array =self.pos_target()



if __name__ == '__main__':

    try:
        X=Transformation()
    except rospy.ROSInterruptException:
        pass
