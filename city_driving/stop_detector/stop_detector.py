import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from detector import StopSignDetector

class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector()
        self.stop_sign_pub = rospy.Publisher("/stop_sign", Bool, queue_size=10)
        self.img_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.img_callback)
        self.depth_sub = rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.depth_callback)
        self.point = None

    def img_callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        is_stop_sign, bounded_box = self.detector.predict(rgb_img)

        if is_stop_sign:
            x_min, y_min, x_max, y_max = tuple(bounded_box)
            pixel = Point()
            pixel.x = int((x_min+x_max)/2)
            pixel.y = int((y_min+y_max)/2)
            self.point = pixel

    def depth_callback(self, img_msg):

        if self.point != None:
            # Logic
            np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
            bgr_img = np_img[:,:,:-1]

            msg = Bool()
            msg.data = True
            self.depth_sub.publish(msg)

            self.point = None



if __name__=="__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()
