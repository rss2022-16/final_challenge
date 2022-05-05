#!/usr/bin/env python

import rospy
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, CompressedImage
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point

from detector import *
from purepursuit import *

# test_img = cv2.imread("/home/racecar/racecar_ws/src/race/race/image3.png")

class LaneFollower():
    """
    A controller to drive in between two lanes
    Extracts lane location from racecar's camera images
    """

    def __init__(self):
        DRIVE_TOPIC = rospy.get_param("~drive_topic") #"/vesc/ackermann_cmd_mux/input/navigation"
        self.vel = rospy.get_param("~vel")
        self.lidar_to_base_axel = rospy.get_param("~lidar_to_base_axel")
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.L = rospy.get_param("~L")
        self.extend_dist = rospy.get_param("~extend_dist")
        self.lane_offset = rospy.get_param("~lane_offset")
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color/compressed", CompressedImage, self.image_callback)
        self.errpub = rospy.Publisher("/error", Float32, queue_size=1)
        
        self.bridge = CvBridge()

        self.pathFinder = Detector(self.lane_offset, self.extend_dist)
  
        self.msg = AckermannDriveStamped()
        self.msg.header.frame_id = "base_link"

    def image_callback(self, msg):
        
        img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        (pt1, pt2) = self.pathFinder.imgtoline(img)
        # pt1, pt2 = self.pathFinder.imgtoline(img)
        # rospy.loginfo(["Line: ", [pt1, pt2]])
        if pt1 is None:
            return

        # x, y = self.extendLine(pt1, pt2)
        waypoints = np.array([pt1, pt2])
        # rospy.loginfo(["Extended: ", [x,y]])

        eta, vel = purepursuit(self.lookahead_distance, self.L, self.vel, 
            self.lidar_to_base_axel, -0.1, 0, waypoints)
        # self.errpub.publish(pt1[1])            
        self.send_drive(eta, vel)

    def extendLine(self, pt1, pt2):
        if pt1[0] < pt2[0]:
            x1 = pt1[0]
            y1 = pt1[1]
            x2 = pt2[0]
            y2 = pt2[1]
        else:
            x1 = pt2[0]
            y1 = pt2[1]
            x2 = pt1[0]
            y2 = pt1[1]

        m = (y2-y1)/(x2-x1)

        x = np.sqrt(self.extend_dist**2/(m**2+1))
        y = m*x 

        return (x1 + x, y1 + y)

    def send_drive(self, eta, vel):
        """
        Helper function
        Sends AckermannDrive msg w/ eta, vel values
        """
        self.msg.header.stamp = rospy.Time.now()
        self.msg.drive.steering_angle = eta
        self.msg.drive.speed = vel
        self.drive_pub.publish(self.msg)


        

if __name__ == '__main__':
    try:
        rospy.init_node('LaneFollower', anonymous=True)
        LaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


    
