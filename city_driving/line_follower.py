#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from pure_pursuit import *

class LineFollower():
    """
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /drive
    """

    VEL = rospy.get_param("~line_follower_velocity", 0.3)  

    # PP Stuff
    LIDAR_TO_BASE_AXEL = -0.35 # Temporary parameter
    LOOKAHEAD_DISTANCE = 1.0
    L = 0.375

    def __init__(self):
        # Subscribe to ZED camera RGB frames
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge()

    def image_callback(self, image_msg):

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        image = image[150:300, :, :]

        waypoints = np.array(([self.LIDAR_TO_BASE_AXEL, 0], \
            [self.LIDAR_TO_BASE_AXEL + 5*self.relative_x, 5*self.relative_y]))

        eta, vel = purepursuit(self.LOOKAHEAD_DISTANCE, self.L, self.VEL, 
            self.LIDAR_TO_BASE_AXEL, 0, 0, waypoints)

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
