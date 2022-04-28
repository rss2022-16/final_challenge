#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
# from msg import ConeLocationPixel
from std_msgs.msg import Bool

# import your color segmentation algorithm; call this function in ros_image_callback!
from color_segmentation import cd_color_segmentation


class LineDetector():
    """
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = True

        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_cone_px", Point, queue_size=10)
        self.line_detector_pub = rospy.Publisher("/line_detector", Bool, queue_size=10)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # Applies imported color segmentation function (cd_color_segmentation) to the image msg here
        # From the bounding box, takes the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # Publishes this pixel (u, v) to the /relative_blue_px topic; the homography transformer will
        # convert it to the car frame.

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        #print (image.shape)
        image = image[215:320, :, :] 
        bbox = cd_color_segmentation(image, None, "orange")
        bottom_pixel = ( (bbox[1][0] + bbox[0][0])/2.0, bbox[1][1])
        pixel = Point()
        #print (bottom_pixel)
        u = bottom_pixel[0] + 0
        v = bottom_pixel[1] + 215
        pixel.y = u
        pixel.x = v
        #print (pixel.v)
        self.cone_pub.publish(pixel)

        detected = Bool()
        if u != 0 or v!= 0:
            detected.data = True
        else:
            detected.data = False
        self.line_detector_pub.publish(detected)

        #image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('LineDetector', anonymous=True)
        LineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
