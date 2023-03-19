#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
import time
from math import pi
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError

class Camera():

    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        self.rate = rospy.Rate(0.1)
        rospy.loginfo("Camera...")
        self._check_camera_ready()
        self.bridge = CvBridge()
        self.obstacle_publisher = rospy.Publisher('/obstacle1', String, queue_size=10)
        # Subscribe to scan
        self.laser_subscriber = rospy.Subscriber('/camera/camera_left/image_raw', Image, self.camera_callback)


    def _check_camera_ready(self):
        camera_msg = None
        rospy.loginfo("Checking Laser...")
        while camera_msg is None and not rospy.is_shutdown():
            try:
                camera_msg = rospy.wait_for_message("/camera/camera_left/image_raw", Image, timeout=1.0)
                rospy.logdebug("Current /camera/image_raw READY=>" + str(camera_msg))

            except:
                rospy.logerr("Current /camera/image_raw not ready yet, retrying for getting camera")
        rospy.loginfo("Checking Camera...DONE")

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

        # Uncomment the following line to save the image as a PNG file
        # cv2.imwrite("image.png", cv_image)
        
        # self.obstacle_publisher.publish(obstacle)


if __name__ == '__main__':
    
    camera = Camera()
    rospy.spin()
    cv2.destroyAllWindows()