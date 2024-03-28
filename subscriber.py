#!/usr/bin/env python
import rospy
from beginner_tutorials.msg import Magnetometer
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

motor_pin = 18  # PWM pin connected to the motor

GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_pin, GPIO.OUT)

motor_pwm = GPIO.PWM(motor_pin, 50)  # 50 Hz
motor_pwm.start(0)  # Start with motor off

status = 0

def call_key(data):
    rospy.loginfo(rospy.get_caller_id() + 'Keyboard %s', data.data)
    print("keyboard: " + str(data.data))
    global status
    if (isinstance(int(str(data.data)), int)):
        status = int(str(data.data))
        if (status > 5):
            status = 0

        if status == 5:
            motor_pwm.ChangeDutyCycle(50)
            time.sleep(1) 
            motor_pwm.ChangeDutyCycle(0)

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
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        motor_pwm.stop()
        GPIO.cleanup()
