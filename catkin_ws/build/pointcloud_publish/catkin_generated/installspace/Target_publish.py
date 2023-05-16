#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from geometry_msgs.msg import Pose
import itertools
import math
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster
from rospy import Time
import numpy as np
import pandas as pd
from scipy.interpolate import make_interp_spline
from scipy.interpolate import interp1d
import sys
from std_msgs.msg import Header

class Target_Trajectory:

    def __init__(self ):
        rospy.init_node('transformed_pointcloud', anonymous=True)
        arg1=rospy.get_param("~arg1")
        #print(arg1)
        trajectory=pd.read_csv(arg1).to_numpy()
        #rospy.loginfo(traj)
        self.rate=rospy.Rate(10)
        self.pose=Pose()
        trajectory_array=[]
        yaw_rate=[]
        trajectory_array =self.pos_target(trajectory)
        #trajectory_array =self.extrpolated_position()
        yaw_temp=self.yaw_rate_calculate(trajectory_array)
        #rospy.loginfo(len(yaw_temp))
        #rospy.loginfo(len(trajectory_array))

        # create a new Header message and set its fields
        
        pos_pub = rospy.Publisher("target_pose", Pose,queue_size=10)
        rospy.loginfo("Initializing Target Vehicle Pose publisher node...")
        
        while not rospy.is_shutdown(): 
            for i,j in zip(trajectory_array,yaw_temp):

                header_msg = Header()
                header_msg.stamp = rospy.Time.now()
                header_msg.frame_id = 'world'
                self.pose.position.x=i[0]
                self.pose.position.y=i[1]
                self.pose.position.z=i[2]
                quaternion = quaternion_from_euler(0, 0, j)
                self.pose.orientation.w=j
                #rospy.loginfo(self.pose.position)
                #rospy.loginfo(j)
                pos_pub.publish(self.pose)
                #translation = (self.pose.position.x, self.pose.position.y, self.pose.position.z)
                #rotation = (quaternion[0], quaternion[1], quaternion[2], quaternion[3])
                #Broadcaster.sendTransform(translation, rotation, Time.now(), 'target', 'point')
                self.rate.sleep()


    def pos_target(self,trajectory):
        """trajectory=np.array([[-25,0,0.6],[-15,0,0.6],[-12,1,0.6],[-9,2,0.6],[-6,3,0.6],[-3,4,0.6],[0,5,0.6],[10,5,0.6],\
                                [12,4,0.6],[15,3,0.6],[18,2,0.6],[21,1,0.6],[23,0,0.6],[25,0,0.6],[23,0,0.6],[21,-1,0.6],[18,-2,0.6],\
                                    [15,-3,0.6],[12,-4,0.6],[10,-5,0.6],[0,-5,0.6],[-3,-4,0.6],[-6,-3,0.6],[-9,-2,0.6],[-12,-1,0.6],[-15,0,0.6],\
                                        [-25,0,0.6],[-15,0,0.6],[-12,-1,0.6],[-9,-2,0.6],[-6,-3,0.6],[-3,-4,0.6],[0,-5,0.6], [10,5,0.6],[12,4,0.6],\
                                            [10,5,0.6],[0,5,0.6],[-3,4,0.6],[-6,3,0.6],[-9,2,0.6],[-12,1,0.6],[-15,0,0.6],[-24,0,0.6]])
        """
        trajectory_extrapolated=[]
        for x in range(len(trajectory)-1):
            trajectory_extrapolated.append(np.linspace(trajectory[x],trajectory[x+1],30)[:-1])
        trajectory_extrapolated=np.array(list(itertools.chain.from_iterable(trajectory_extrapolated)))
        return trajectory_extrapolated

    def extrpolated_position(self):

        a = 2
        b = 3
        r = 10

        #The lower this value the higher quality the circle is with more points generated
        stepSize = 0.06

        #Generated vertices
        positions = []

        t = 0
        while t < 2 * math.pi:
            positions.append((r * math.cos(t) + a, r * math.sin(t) + b,0.6))
            t += stepSize

        return positions

    def yaw_rate_calculate(self, trajectory):
        yaw_rate=[]
        for x in range(len(trajectory)-1):
            x1=trajectory[x][0]
            x2=trajectory[x+1][0]
            y1=trajectory[x][1]
            y2=trajectory[x+1][1]
            angle=math.degrees(math.atan(abs((y1-y2))/abs((x1-x2))))

            if y1<y2:
                if x1<x2:
                    yaw_rate_temp=angle
                else:
                    yaw_rate_temp=180-angle
            elif y1>y2:
                if x1<x2:
                    yaw_rate_temp=360-angle
                else:
                    yaw_rate_temp=180+angle  
            else:
                if x1>x2:
                    yaw_rate_temp=180+angle
                else:
                    yaw_rate_temp=angle 
                #yaw_rate_temp=angle

            yaw_rate.append(yaw_rate_temp)
            #rospy.loginfo(yaw_rate_temp)
        return yaw_rate

if __name__ == '__main__':

    try:
        
        X=Target_Trajectory()
    except rospy.ROSInterruptException:
        pass