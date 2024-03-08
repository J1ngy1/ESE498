#!/usr/bin/env python
import rospy
from  beginner_tutorials.msg import Magnetometer

from sensor_msgs.msg import LaserScan

def callback(data):
    if data.ranges:
        rospy.loginfo('Received LiDAR data at the back: %f',data.ranges[100])
    else:
        rospy.loginfo('Out of scan range')

def listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.Subscriber('magnetometer_data', Magnetometer, mag_callback)
    rospy.spin()

def mag_callback(data):
    rospy.loginfo("Magnetometer Data: Mag=(%f, %f, %f)", data.mag_x, data.mag_y, data.mag_z)

def listen_to_sensors():
    rospy.init_node('sensor_subscriber', anonymous=True)
    rospy.Subscriber('magnetometer_data', Magnetometer, mag_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

