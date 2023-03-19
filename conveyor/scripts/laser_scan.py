#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time
from math import pi
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String

class Laser():

    def __init__(self):
        rospy.init_node('laser_scan_node', anonymous=True)
        self.rate = rospy.Rate(10)
        rospy.loginfo("Laser Scan...")
        self._check_laser_ready()
        self.obstacle_publisher = rospy.Publisher('/obstacle', String, queue_size=10)
        # Subscribe to scan
        self.laser_subscriber = rospy.Subscriber('/lidar/scan', LaserScan, self.laser_callback)


    def _check_laser_ready(self):
        laser_msg = None
        rospy.loginfo("Checking Laser...")
        while laser_msg is None and not rospy.is_shutdown():
            try:
                laser_msg = rospy.wait_for_message("/lidar/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /lidar/scan READY=>" + str(laser_msg))

            except:
                rospy.logerr("Current /lidar/scan not ready yet, retrying for getting scan")
        rospy.loginfo("Checking Laser...DONE")

    def laser_callback(self, msg):
        rospy.loginfo(f"Length of msg (expected 30): {len(msg.ranges)}")
        rospy.loginfo(f"Length of msg at 15: {msg.ranges[15]}")
        
        # self.obstacle_publisher.publish(obstacle)


if __name__ == '__main__':
    
    laser = Laser()
    while not rospy.is_shutdown():
        try:
            laser.rate.sleep()
        except rospy.ROSInterruptException:
            pass