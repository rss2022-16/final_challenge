import cv2
import numpy as np
import scipy
import math

height = 1080
width = 1920
mid = int(width/2)

img = cv2.imread('lane.jpg')
img = cv2.resize(img, (1920, 1080))
grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.namedWindow("out", cv2.WINDOW_NORMAL)


#filter only the white portion of the image and mask everything else
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower_white = np.array([0,0,168])
upper_white = np.array([172,111,255])
mask = cv2.inRange(hsv, lower_white, upper_white)
res = cv2.bitwise_and(img, img, mask= mask)


canny = cv2.Canny(mask, 50, 200, None, 3)
color_canny = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)


# regular hough transform, does not work as well 
#
# lines = cv2.HoughLines(canny, 1, np.pi/180, 250, None, 0, 0)
# if lines is not None:
#     for i in range(0, len(lines)):
#         rho = lines[i][0][0]
#         theta = lines[i][0][1]
#         a = math.cos(theta)
#         b = math.sin(theta)
#         x0 = a * rho
#         y0 = b * rho
#         pt1 = (int(x0 + 1000.0*(-b)), int(y0 + 1000.0*(a)))
#         pt2 = (int(x0 - 1000.0*(-b)), int(y0 - 1000.0*(a)))
#         cv2.line(color_canny, pt1, pt2, (0,0,255), 1, cv2.LINE_AA)

#find all straight lines and mark them red
linesP = cv2.HoughLinesP(canny, 1, np.pi / 180, 50, None, 80, 15)
left = linesP[0][0]
right = linesP[1][0]
if linesP is not None:
    for i in range(0, len(linesP)):
        l = linesP[i][0]
        bot_y = max(l[1], l[3])
        if(l[0] <= mid): 
            if(l[0] > left[0] and bot_y > max(left[1], left[3])):
                left = l
        else:
            if(l[0] < right[0] and bot_y > max(left[1], left[3])):
                right = l
            
        cv2.line(color_canny, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2, cv2.LINE_AA)

#mark "closest" two lines in green
cv2.line(color_canny, (left[0], left[1]), (left[2], left[3]), (0,255,0), 2, cv2.LINE_AA)
cv2.line(color_canny, (right[0], right[1]), (right[2], right[3]), (0,255,0), 2, cv2.LINE_AA)





# cv2.imshow('out', canny)
# cv2.waitKey(0)
cv2.imshow('out', color_canny)
cv2.waitKey(0)

