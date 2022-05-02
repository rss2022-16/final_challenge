#!/usr/bin/env python

import numpy as np
import cv2

class Detector():

    ## Homography
    PTS_IMAGE_PLANE = [[11.0, 329.0],
                       [603.0, 304.0],
                       [43.0, 172.0],
                       [560.0, 162.0]] 
    PTS_GROUND_PLANE = [[12.1, 16.2],
                        [14.25, -9.6],
                        [88.0, 77.25],
                        [110.0, -75.0]]
    METERS_PER_INCH = 0.0254

    ## CV
    height, width = 376, 672
    mid = int(width/2)
    kernel_size = 7
    lower_orange = np.array([4, 150, 100])
    upper_orange = np.array([25, 255, 255])
    ymin, ymax = 150, 300
    
    def __init__(self):

        # cv2.namedWindow("out", cv2.WINDOW_NORMAL)

        #Initialize data into a homography matrix
        np_pts_ground = np.array(self.PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * self.METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])
        np_pts_image = np.array(self.PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])
        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

    def imgtopath(self, img):
        """
        Given a cv image type, return a series of detected lines in (x,y) waypoints
        Returns: List of line segments [ ((x1,y1),(x2,y2)), ... ]
        """
        img = cv2.resize(img, (self.width, self.height))

        # Blur
        blurred = cv2.GaussianBlur(img,(self.kernel_size, self.kernel_size),0)

        # Color Filtering
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)

        # Edge Detection
        canny = cv2.Canny(mask, 50, 200, None, 3)
        # color_canny = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)

        # Crop
        cropped = np.zeros(canny.shape,np.uint8)
        cropped[self.ymin:self.ymax,:] = canny[self.ymin:self.ymax,:]

        # Hough Transform
        linesP = cv2.HoughLinesP(cropped, 1, np.pi / 180, 80, None, 80, 10)
        if linesP is None:
            return None
        # for line in linesP:
        #     l = line[0]
        #     cv2.line(color_canny, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

        # Homography
        reallines = []
        for line in linesP:
            l = line[0]
            if l[3] > l[1]:
                l[3], l[1] = l[1], l[3]
                l[2], l[0] = l[0], l[2]

            (x1, y1) = self.transform(l[0], l[1])
            (x2, y2) = self.transform(l[2], l[3])
            reallines.append((x1,y1,x2,y2))

        return reallines
        

    def transform(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.
        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.
        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y

if __name__ == "__main__":

    img = cv2.imread("lane.jpg")
    detector = Detector()
    lines = detector.imgtopath(img)