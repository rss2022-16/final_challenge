#!/usr/bin/env python

from locale import currency
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from nav_msgs.msg import Odometry
import rospkg
import time, os
from utils import LineTrajectory
import tf
from scipy.ndimage import binary_dilation, generate_binary_structure, iterate_structure
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs import Bool

class CityDriver(object):
    """ 
    """

    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.line_follower_sub = rospy.Subscriber("/line_drive", AckermannDriveStamped, self.line_follower_cb)
        self.wall_follower_sub = rospy.Subscriber("/wall_drive", AckermannDriveStamped, self.wall_follower_cb, queue_size=10)
        self.stop_sign_sub = rospy.Subscriber("/stop_sign", Bool, self.stop_sign_cb)
        self.car_wash__detector_sub = rospy.Subscriber("/car_wash_detector", Bool, self.car_wash_cb)
        self.safety_sub = rospy.Subscriber("/safety", Odometry, self.odom_cb)

        DRIVE_TOPIC = rospy.get_param("~drive_topic")
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

        self.state = "Default"


    def line_follower_cb(self, msg):
        if self.state == "Default":
            self.drive_pub.publish(msg)

    def wall_follower_cb(self, msg):
        if self.state == "in_car_wash":
            self.drive_pub.publish(msg)

    def stop_sign_cb(self, msg):
        # only called when stop_sign detector detects sign close enough
        self.state = "Stop"
        begin_time = rospy.Time.now()
        self.stop_cb(begin_time)

    def car_wash_cb(self, msg):
        # only called when car wash is detected
        self.state = "car_wash_detected"
        self.blue_follower()

    def stop_cb(self, begin_time):
        while rospy.Time.now() - begin_time < 3:
            self.send_drive(0, 0)
        self.state = "Default"
    
    def send_drive(self, v, eta):
        """
        Helper function -- sends v, eta to the car
        """
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.drive.steering_angle = eta
        msg.drive.speed = v
        self.drive_pub.publish(msg)
        

    
    


if __name__=="__main__":
    rospy.init_node("city_driving")
    cd = CityDriver()
    rospy.spin()