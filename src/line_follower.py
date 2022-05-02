#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from pure_pursuit import *
from detector import *

class LineFollower():
    """
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /drive
    """

    def __init__(self):
        # Subscribe to ZED camera RGB frames
        DRIVE_TOPIC = rospy.get_param("~drive_topic") #"/vesc/ackermann_cmd_mux/input/navigation"
        self.vel = rospy.get_param("~vel")
        self.lidar_to_base_axel = rospy.get_param("~lidar_to_base_axel")
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.L = rospy.get_param("~L")

        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge()

        self.pathFinder = Detector()

    def image_callback(self, image_msg):

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        lines = self.pathFinder.imgtopath(img)

        if lines is None:
            rospy.loginfo("No lines detected")
            return

        eta, vel = purepursuit(self.lookahead_distance, self.L, self.vel, 
            self.lidar_to_base_axel, 0, 0, lines)

        if eta is None:
            rospy.loginfo("No purepursuit intersection found")
            return

        self.send_drive(eta, vel)

    def send_drive(self, eta, vel):
        """
        Helper function
        Sends AckermannDrive msg w/ eta, vel values
        """
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.drive.steering_angle = eta
        msg.drive.speed = vel
        self.last_drive = msg
        self.drive_pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('LineFollower', anonymous=True)
        node = LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass