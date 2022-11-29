#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray


def pos_ego():

    rospy.init_node('position_temp')
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing a position")
        pos_pub = rospy.Publisher("position_info", Float32MultiArray)
        posinfo=Float32MultiArray()
        posinfo.data=[10,5,0.598]
        rospy.loginfo("Initializing sample pos publisher node...")
        pos_pub.publish(posinfo)
        rate.sleep()

if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''
    try:
        pos_ego()
    except rospy.ROSInterruptException:
        pass