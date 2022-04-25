import cv2
import numpy as np
import matplotlib.pyplot as plt

height = 376
width = 672
mid = int(width/2)

def euclid_distance(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def cartesian2polar(pt):
    pt = (pt[0]-mid, -(pt[1]-height))
    # (theta, radius)
    return (np.arctan(pt[1]/pt[0]), euclid_distance((0,0), pt))

def orient_line_up(line):
    if line[3] > line[1]: # If endpoint is below start point
        line[0], line[2] = line[2], line[0]
        line[1], line[3] = line[3], line[1]
    return line

# img = cv2.imread('lane.jpg')
test_number = 8
img = cv2.imread(f'TestImagesTrack/test{test_number}.jpg')
img = cv2.resize(img, (width, height))
cv2.namedWindow("out", cv2.WINDOW_NORMAL)

# Blur
kernel_size = 7
blurred = cv2.GaussianBlur(img,(kernel_size, kernel_size),0)

# Color Filtering
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
lower_white = np.array([0,0,168])
upper_white = np.array([172,111,255])
mask = cv2.inRange(hsv, lower_white, upper_white)

# Edge Detection
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


linesP = cv2.HoughLinesP(canny, 1, np.pi / 180, 50, None, 80, 10)
left = linesP[0][0]
right = linesP[1][0]

#filter out horizontal lines
slope_thresh = np.pi/12
vertical_lines = []
for line in linesP:
    l = line[0]
    angle = np.arctan2(abs(l[3]-l[1]), l[2]-l[0])
    if(slope_thresh < angle < np.pi - slope_thresh):
        vertical_lines.append(line)

left = None
right = None

for i in range(0, len(vertical_lines)):
    l = vertical_lines[i][0]
    bot_y = max(l[1], l[3])
    slope = np.arctan2(l[3]-l[1], l[2]-l[0])

    l = orient_line_up(l) #make sure start pt is on bottom

    # if min(l[0], l[2])  <= mid:
    if l[0] <= mid:
        if left is None:
            left = l
        # closest by polar, theta then radius
        else:
            l_end_polar = cartesian2polar((l[0], l[1]))
            left_end_polar = cartesian2polar((left[0], left[1]))
            if l_end_polar[0] > left_end_polar[0]:
                left = l

        # # closest to (mid, height)
        # else:
        #     l_dist = euclid_distance((mid, height), (l[0], l[1]))
        #     left_dist = euclid_distance((mid, height), (left[0], left[1]))
        #     if l_dist < left_dist:
        #         left = l
        # # lowest endpoint
        # elif min(l[0], l[2]) > min(left[0], left[2]):
        #     left = l
    else:
        if right is None:
            right = l
        # closest by polar, theta then radius
        else:
            l_end_polar = cartesian2polar((l[0], l[1]))
            right_end_polar = cartesian2polar((right[0], right[1]))
            if l_end_polar[0] < right_end_polar[0]:
                right = l
        # # closest to (mid, height)
        # else:
        #     l_dist = euclid_distance((mid, height), (l[0], l[1]))
        #     right_dist = euclid_distance((mid, height), (right[0], right[1]))
        #     if l_dist < right_dist:
        #         right = l
        # # lowest endpoint
        # if max(l[0], l[2]) < max(right[0], right[2]):
        #     right = l

    cv2.line(color_canny, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
    # cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

#mark "closest" two lines in green
cv2.line(color_canny, (left[0], left[1]), (left[2], left[3]), (0,255,0), 3, cv2.LINE_AA)
cv2.line(color_canny, (right[0], right[1]), (right[2], right[3]), (0,255,0), 3, cv2.LINE_AA)
# cv2.line(img, (left[0], left[1]), (left[2], left[3]), (0,255,0), 3, cv2.LINE_AA)
# cv2.line(img, (right[0], right[1]), (right[2], right[3]), (0,255,0), 3, cv2.LINE_AA)


if left[3] > left[1]: # If endpoint is below start point
    left[0], left[2] = left[2], left[0]
    left[1], left[3] = left[3], left[1]

if right[3] > right[1]:
    right[0], right[2] = right[2], right[0]
    right[1], right[3] = right[3], right[1]

bot_x = int(left[0]/2 + right[0]/2)
bot_y = int(left[1]/2 + right[1]/2)
top_x = int(left[2]/2 + right[2]/2)
top_y = int(left[3]/2 + right[3]/2)

cv2.line(color_canny, (bot_x, bot_y), (top_x, top_y), (255,0,0), 3, cv2.LINE_AA)
# cv2.line(img, (bot_x, bot_y), (top_x, top_y), (255,0,0), 3, cv2.LINE_AA)

# cv2.imshow('out', canny)
# cv2.waitKey(0)
cv2.imshow('out', color_canny)
# cv2.imshow('out', img)
cv2.waitKey(0)
