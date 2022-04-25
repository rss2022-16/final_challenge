import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from detector import StopSignDetector

class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector()
        self.stop_sign_pub = rospy.Publisher("/relative_cone_px", Point, queue_size=10)
        self.img_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)

    def callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        is_stop_sign, bounded_box = self.detector.predict(rgb_img)

        if is_stop_sign:
            x_min, y_min, x_max, y_max = tuple(bounded_box)
            pixel = Point()
            pixel.x = int((x_min+x_max)/2)
            pixel.y = y_min
            self.stop_sign_pub.publish(pixel)


if __name__=="__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()
