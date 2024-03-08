#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from beginner_tutorials.msg import Magnetometer
import board
import busio
import adafruit_mmc65x3
import adafruit_mpu6050

def publish_sensor_data():
    imu_pub = rospy.Publisher('imu_data', Imu, queue_size=10)
    mag_pub = rospy.Publisher('magnetometer_data', Magnetometer, queue_size=10)
    rospy.init_node('sensor_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    # Initialize I2C bus and sensors
    i2c = busio.I2C(board.SCL, board.SDA)
    mmc = adafruit_mmc65x3.MMC65X3(i2c)
    mpu = adafruit_mpu6050.MPU6050(i2c)

    while not rospy.is_shutdown():
        imu_msg = Imu()
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = mpu.acceleration
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = mpu.gyro

        mag_msg = Magnetometer()
        mag_msg.mag_x, mag_msg.mag_y, mag_msg.mag_z = mmc.magnetic

        rospy.loginfo(imu_msg)
        imu_pub.publish(imu_msg)

        rospy.loginfo(mag_msg)
        mag_pub.publish(mag_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_sensor_data()
    except rospy.ROSInterruptException:
        pass
