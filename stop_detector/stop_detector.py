import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped

class SignDetector:
    def __init__(self):
        self.stop_sign_pub = rospy.Publisher("/stop_sign", Bool, queue_size=10)
        self.img_sub = rospy.Subscriber("/stop_sign_bbox", Float32MultiArray, self.img_callback)
        self.depth_sub = rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.depth_callback)
        self.stop_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size = 10)
        self.point = None
        self.low_depth = 110.0
        self.high_depth = 190.0
        self.last_stop = None

    def img_callback(self, img_msg):
        bounded_box = img_msg.data 
        x_min, y_min, x_max, y_max = tuple(bounded_box)
        pixel = Point()
        pixel.x = int((x_min+x_max)/2)
        pixel.y = int((y_min+y_max)/2)
        self.point = pixel

    def depth_callback(self, img_msg):
        if self.point != None and (self.last_stop == None or (rospy.Time.now() - self.last_stop).to_sec() > 5):
            gray_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)

            #gray_img = np_img[:,:,:-1]
            #print (image.shape)
            x = self.point.x
            y = self.point.y
            gray_value = gray_img[y, x, 2]
            if gray_value >= self.low_depth and gray_value <= self.high_depth:
                print (gray_value)
                msg = Bool()
                msg.data = True
                self.last_stop = rospy.Time.now()
                self.stop_sign_pub.publish(msg)
                
                time = rospy.Time.now()
                while (rospy.Time.now() - time).to_sec() < 3:
                    drive_msg = AckermannDriveStamped()
                    drive_msg.header.stamp = rospy.Time.now()
                    drive_msg.header.frame_id = "base_link"
                    drive_msg.drive.speed = 0
                    self.stop_pub.publish(drive_msg)
                    rospy.loginfo("PUBLUSHED INFO")

                self.point = None



if __name__=="__main__":
    rospy.init_node("sign_detector")
    detect = SignDetector()
    rospy.spin()
