#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from beginner_tutorials.msg import Magnetometer
import board
import busio
import adafruit_mmc56x3

def publish_sensor_data():
    mag_pub = rospy.Publisher('magnetometer_data', Magnetometer, queue_size=10)
    rospy.init_node('sensor_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    # Initialize I2C bus and sensors
    i2c = busio.I2C()
    mmc = adafruit_mmc56x3.MMC5603(i2c)

    while not rospy.is_shutdown():
        mag_msg = Magnetometer()
        mag_msg.mag_x, mag_msg.mag_y, mag_msg.mag_z = mmc.magnetic

        rospy.loginfo(mag_msg)
        mag_pub.publish(mag_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_sensor_data()
    except rospy.ROSInterruptException:
        pass
