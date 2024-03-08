#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    if data.ranges:
        rospy.loginfo('Received LiDAR data at the back: %f',data.ranges[100])
    else:
        rospy.loginfo('Out of scan range')

def listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
