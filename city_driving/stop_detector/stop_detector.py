import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from visual_servoing.msg import ConeLocationPixel
from detector import StopSignDetector

class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector()
        self.stop_sign_pub = rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.img_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)

    def callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        #TODO: 
        is_stop_sign, bounded_box = self.detector.predict(rgb_img)

        if is_stop_sign:
            x_min, y_min, x_max, y_max = tuple(bounded_box)
            img_spliced = bgr_img[y_min: y_max, x_min:x_max]
            if self.is_octagon(img_spliced):
                pixel = ConeLocationPixel()
                pixel.u = y_min
                pixel.v = int((x_min+x_max)/2)
                self.stop_sign_pub.publish(pixel)

    def is_octagon(self, img):
        img_contour = img.copy()
        img_blur = cv2.GaussianBlur(img, (7,7), 1)
        img_gray = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)
        img_canny = cv2.Canny(img_gray, 100, 200)

        kernel = np.ones((5,5))
        img_dilate = cv2.dilate(img_canny, kernel, iterations = 1)

        # cv2.imshow('dilate', img_dilate)

        contours, _ = cv2.findContours(img_dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:
                cv2.drawContours(img_contour, cnt, -1, (255,0,255),7)
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
                # if the number of vertices is 8, then the shape is an octagon
                if len(approx) == 8:
                    # Useful for debugging (draws rectangle around shapes)
                    #
                    # x, y, w, h = cv2.boundingRect(approx)
                    # cv2.rectangle(img_contour, (x,y), (x + w, y+h),(0,255,0), 5)
                    # cv2.putText(img_contour, "octagon", (x+w + 20, y + 20), 
                    #             cv2.FONT_HERSHEY_COMPLEX, .7, (0,255,0), 2)
                    return True

        return False





if __name__=="__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()
