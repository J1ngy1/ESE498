#!/usr/bin/env python
import rospy
from beginner_tutorials.msg import Magnetometer
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
import math
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
status = 0

def Servo_Motor_Initialization():
    i2c_bus = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c_bus)
    pca.frequency = 100
    return pca
    
def Motor_Speed(pca, percent):
    speed = (percent * 3277) + 65535 * 0.15
    pca.channels[15].duty_cycle = math.floor(speed)

pca = Servo_Motor_Initialization()
Motor_Speed(pca, 0) 
channel_num = 14
servo7 = servo.Servo(pca.channels[channel_num])
    
def call_key(data):
    rospy.loginfo(rospy.get_caller_id() + 'Keyboard %s', data.data)
    print("keyboard: " + str(data.data))
    global status
    if (isinstance(int(str(data.data)), int)):
        status = int(str(data.data))
        if (status > 8):
            status = 0

    if status == 5:
        Motor_Speed(pca, 0.15)
        time.sleep(1)  # Run for 1 second
        Motor_Speed(pca, 0)
        status = 0
    elif status == 6:
        servo7.angle = 45
        time.sleep(0.5)
        status = 0
    elif status == 7:
        servo7.angle = 135        
        time.sleep(0.5)
        status = 0
    elif status == 8:
        servo7.angle = 90        
        time.sleep(0.5)
        status = 0

def call_lidar(data):
    if (status == 1 or status == 2):
        if data.ranges:
            rospy.loginfo('Received LiDAR data at the back: %f',data.ranges[100])
        else:
            rospy.loginfo('Out of scan range')
            
def imu_callback(data):
    if (status == 1 or status == 3):
        rospy.loginfo("IMU Data:")
        rospy.loginfo("Linear Acceleration - x: {:.2f}, y: {:.2f}, z: {:.2f}".format(
            data.linear_acceleration.x,
            data.linear_acceleration.y,
            data.linear_acceleration.z))
        rospy.loginfo("Angular Velocity - x: {:.2f}, y: {:.2f}, z: {:.2f}".format(
            data.angular_velocity.x,
            data.angular_velocity.y,
            data.angular_velocity.z))           

def mag_callback(data):
    if (status == 1 or status == 4):
        rospy.loginfo("Magnetometer Data: Mag=(%f, %f, %f)", data.mag_x, data.mag_y, data.mag_z)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, call_lidar)
    rospy.Subscriber('keyboard', String, call_key)
    rospy.Subscriber('magnetometer_data', Magnetometer, mag_callback)
    rospy.Subscriber('imu/data', Imu, imu_callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
