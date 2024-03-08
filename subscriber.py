#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from  beginner_tutorials.msg import Magnetometer

def imu_callback(data):
    rospy.loginfo("IMU Data: Accel=(%f, %f, %f), Gyro=(%f, %f, %f)", data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z, data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)

def mag_callback(data):
    rospy.loginfo("Magnetometer Data: Mag=(%f, %f, %f)", data.mag_x, data.mag_y, data.mag_z)

def listen_to_sensors():
    rospy.init_node('sensor_subscriber', anonymous=True)
    rospy.Subscriber('imu_data', Imu, imu_callback)
    rospy.Subscriber('magnetometer_data', Magnetometer, mag_callback)
    rospy.spin()

if __name__ == '__main__':
    listen_to_sensors()
