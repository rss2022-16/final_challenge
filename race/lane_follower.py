#!/usr/bin/env python

import rospy
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point

from image_processing import *

class LaneFollower():
    """
    A controller to drive in between two lanes
    Extracts lane location from racecar's camera images
    """
    

    def __init__(self):
        DRIVE_TOPIC = rospy.get_param("~drive_topic") #"/vesc/ackermann_cmd_mux/input/navigation"
        #VEL = rospy.get_param("lane_follower/velocity")
        VEL = 2

        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)

        self.vel = VEL
        self.bridge = CvBridge()


    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # image processing to find line 

        # homography to transform line into points

        # pure pursuit to follow the points

if __name__ == '__main__':
    try:
        rospy.init_node('LaneFollower', anonymous=True)
        LaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


    
