#!/usr/bin/env python
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
from scipy.interpolate import make_interp_spline


class Target_Trajectory:

    def __init__(self):
        rospy.init_node('transformed_pointcloud', anonymous=True)
        self.rate=rospy.Rate(10)
        self.pose=Pose()
        trajectory_array=[]
        yaw_rate=[]
        Broadcaster = TransformBroadcaster()
        #trajectory_array =self.extrpolated_position()
        trajectory_array =self.pos_target()
        yaw_temp=self.yaw_rate_calculate(trajectory_array)
        rospy.loginfo(len(yaw_temp))
        rospy.loginfo(len(trajectory_array))
        pos_pub = rospy.Publisher("target_pose", Pose,queue_size=10)
        rospy.loginfo("Initializing sample pos publisher node...")
        
        while not rospy.is_shutdown(): 
            for i,j in zip(trajectory_array,yaw_temp):
                self.pose.position.x=i[0]
                self.pose.position.y=i[1]
                self.pose.position.z=i[2]
                quaternion = quaternion_from_euler(0, 0, j)
                self.pose.orientation.w=j
                """rospy.loginfo(yaw_temp)
                rospy.loginfo(len(yaw_temp))
                rospy.loginfo(len(trajectory_array))"""
                rospy.loginfo(self.pose)
                pos_pub.publish(self.pose)
                translation = (self.pose.position.x, self.pose.position.y, self.pose.position.z)
                
                rotation = (quaternion[0], quaternion[1], quaternion[2], quaternion[3])
                Broadcaster.sendTransform(translation, rotation, Time.now(), 'target', 'point')
                self.rate.sleep()


    def pos_target(self):
        trajectory=np.array([[-25,0,0.6],[-15,0,0.6],[-12,1,0.6],[-9,2,0.6],[-6,3,0.6],[-3,4,0.6],[0,5,0.6],[10,5,0.6],\
                                [12,4,0.6],[15,3,0.6],[18,2,0.6],[21,1,0.6],[23,0,0.6],[25,0,0.6]])
        trajectory_extrapolated=[]
        for x in range(len(trajectory)-1):
            trajectory_extrapolated.append(np.linspace(trajectory[x],trajectory[x+1],30)[:-1])
            #yaw_rate_temp=math.degrees(math.atan((trajectory[x][1]-trajectory[x+1][1])/(trajectory[x][0]-trajectory[x+1][0])))
            #yaw_rate.append(yaw_rate_temp)
        #rospy.loginfo(yaw_rate)
        trajectory_extrapolated=np.array(list(itertools.chain.from_iterable(trajectory_extrapolated)))
        return trajectory_extrapolated

    def extrpolated_position(self):

        # Dataset
        x = np.array([-10, -9, 9, 10])
        y = np.array([ 0, 2, 2, 0])

        X_Y_Spline = make_interp_spline(x, y)

        # Returns evenly spaced numbers
        # over a specified interval.
        X_ = np.linspace(x.min(), x.max(), 1000)
        Y_ = X_Y_Spline(X_)

        x1=np.linspace(-25,-10,1000)
        x2=np.linspace(10,25,1000)
        y0=np.zeros(1000)

        xf1=np.append(x1,X_)
        yf2=np.append(y0,Y_)

        xf=np.append(xf1,x2)
        yf=np.append(yf2,y0)
        zf=np.full(len(xf),0.6)

        #final1=list(zip(xf,yf,zf))
        lst=list(zip(xf,yf,zf))
        final1=list(set([i for i in lst]))
        final = np.asarray([ list(a) for a in final1])

        return final
    
    def yaw_rate_calculate(self, trajectory):
        yaw_rate=[]
        for x in range(len(trajectory)-1):
            yaw_rate_temp=math.degrees(math.atan((trajectory[x][1]-trajectory[x+1][1])/(trajectory[x][0]-trajectory[x+1][0])))
            yaw_rate.append(yaw_rate_temp)
        rospy.loginfo(yaw_rate)
        return yaw_rate



if __name__ == '__main__':

    try:
        X=Target_Trajectory()
    except rospy.ROSInterruptException:
        pass