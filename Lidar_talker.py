import rospy
from sensor_msgs.msg import LaserScan
from adafruit_rplidar import RPLidar

def talker():
    pub = rospy.Publisher('lidar_data', LaserScan, queue_size=10)
    rospy.init_node('lidar_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    PORT_NAME = '/dev/ttyUSB0'
    lidar = RPLidar(None, PORT_NAME, timeout=3)

    while not rospy.is_shutdown():
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = 0.0
        scan.angle_max = 2 * pi
        scan.angle_increment = 2 * pi / 360
        scan.time_increment = 0
        scan.range_min = 0.0
        scan.range_max = 5.0

        measurements = []
        for (_, angle, distance) in lidar.iter_measures():
            measurements.append(distance / 1000.0)  # meters
        scan.ranges = measurements

        rospy.loginfo('Publishing LiDAR data')
        pub.publish(scan)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    finally:
        lidar.stop()
        lidar.disconnect()
