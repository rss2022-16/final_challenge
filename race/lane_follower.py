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
from purepursuit import *

class LaneFollower():
    """
    A controller to drive in between two lanes
    Extracts lane location from racecar's camera images
    """
    LIDAR_TO_BASE_AXEL = -0.35 
    LOOKAHEAD_DISTANCE = .6
    L = 0.375
    VEL = 2
    EXDEND_D = 10

    def __init__(self):
        DRIVE_TOPIC = rospy.get_param("~drive_topic") #"/vesc/ackermann_cmd_mux/input/navigation"
        #VEL = rospy.get_param("lane_follower/velocity")
        

        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)

        self.vel = self.VEL
        self.bridge = CvBridge()


    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        pathFinder = Detector()

        pt1, pt2 = pathFinder.imgtoline(img)
        x, y = self.extendLine(pt1, pt2)

        if x is not None:

            waypoints = np.array(([self.LIDAR_TO_BASE_AXEL, 0], \
                    [self.LIDAR_TO_BASE_AXEL + x, y]))

            eta, vel = purepursuit(self.LOOKAHEAD_DISTANCE, self.L, self.VEL, 
                self.LIDAR_TO_BASE_AXEL, 0, 0, waypoints)
            
            self.send_drive(eta, vel)

    def extendLine(self, pt1, pt2):
        if pt1[1] < pt2[1]:
            x1 = pt1[0]
            y1 = pt1[1]
            x2 = pt2[0]
            y2 = pt2[2]
        else:
            x1 = pt2[0]
            y1 = pt2[1]
            x2 = pt1[0]
            y2 = pt1[2]

        m = (y2-y1)/(x2-x1)

        x = np.sqrt(self.EXDEND_D**2/(m**2+1))
        y = m*x 

        return (x1+x, y1+y)

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
        self.drive_pub.publish(msg)


        

if __name__ == '__main__':
    try:
        rospy.init_node('LaneFollower', anonymous=True)
        LaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


    
