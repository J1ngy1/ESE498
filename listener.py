import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    rospy.loginfo('Received LiDAR data:')
    for i, range in enumerate(data.ranges):
        rospy.loginfo('Angle: %f, Distance: %f', data.angle_min + i * data.angle_increment, range)

def listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber('lidar_data', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()