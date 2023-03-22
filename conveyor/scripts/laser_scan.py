#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from gazebo_conveyor.msg import ConveyorBeltState

class Laser():

    def __init__(self):
        rospy.init_node('laser_scan_node', anonymous=True)
        self.rate = rospy.Rate(30)
        rospy.loginfo("Laser Scan...")
        self._check_laser_ready()
        self.object_detect = False
        self.timeout = rospy.Duration(1) #time taken for object to travel from laser scan to camera at 0.3 m/s
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
                rospy.logdebug("Current /lidar/scan not ready yet, retrying for getting scan")
        rospy.loginfo("Checking Laser...DONE")

    def laser_callback(self, msg):
        self.rate.sleep()
        if msg.ranges[15] <=0.5 and self.object_detect is False:
            self.object_detect = True
            rospy.Timer(self.timeout,self._publish_to_camera_callback,True)
            
        elif msg.ranges[15] >0.5 and self.object_detect is True:
            self.object_detect = False
        

    def _publish_to_camera_callback(self,event):
        self.obstacle_publisher.publish("In")


if __name__ == '__main__':
    
    laser = Laser()
    while not rospy.is_shutdown():
        try:
            laser.rate.sleep()
        except rospy.ROSInterruptException:
            pass