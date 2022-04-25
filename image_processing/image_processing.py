import cv2
import numpy as np
import matplotlib.pyplot as plt

height = 376
width = 672
mid = int(width/2)

## Homography

######################################################
PTS_IMAGE_PLANE = [[11.0, 329.0],
                   [603.0, 304.0],
                   [43.0, 172.0],
                   [560.0, 162.0]] 
######################################################

######################################################
PTS_GROUND_PLANE = [[12.1, 16.2],
                    [14.25, -9.6],
                    [88.0, 77.25],
                    [110.0, -75.0]]
######################################################

METERS_PER_INCH = 0.0254

def transform(h, u, v):
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
    xy = np.dot(h, homogeneous_point)
    scaling_factor = 1.0 / xy[2, 0]
    homogeneous_xy = xy * scaling_factor
    x = homogeneous_xy[0, 0]
    y = homogeneous_xy[1, 0]
    return x, y

# self.drawing = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color_mouse_left", Point, self.call_marker)

#Initialize data into a homography matrix

np_pts_ground = np.array(PTS_GROUND_PLANE)
np_pts_ground = np_pts_ground * METERS_PER_INCH
np_pts_ground = np.flcv2.line(color_canny, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)oat32(np_pts_ground[:, np.newaxis, :])

np_pts_image = np.array(PTS_IMAGE_PLANE)
np_pts_image = np_pts_image * 1.0
np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

h, err = cv2.findHomography(np_pts_image, np_pts_ground)

## Cv2 stuff

img = cv2.imread('assets/test10.jpg')
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
y0, yf = int(2*height/5), height
cropped = np.zeros(canny.shape,np.uint8)
cropped[y0:yf,:] = canny[y0:yf,:]
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

# Hough Transform
linesP = cv2.HoughLinesP(cropped, 1, np.pi / 180, 50, None, 80, 10)

# Homography
reallines = []
for line in linesP:
    l = line[0]
    if l[3] > l[1]:
        l[3], l[1] = l[1], l[3]
        l[2], l[0] = l[0], l[2]

    (x1, y1) = transform(h, l[0], l[1])
    (x2, y2) = transform(h, l[2], l[3])
    reallines.append((x1,y1,x2,y2))

# uvdist = []
# uvang = []
# xydist = []
# xyang = []
# for i in range(len(linesP)):
#     l = linesP[i][0]
#     if min(l[3],l[1]) > 2*height/5: # Ignore top ~half of image
#         if l[3] > l[1]:
#             dist = np.sqrt((l[2]-width/2)**2 + (l[3]-height)**2)
#         else:
#             dist = np.sqrt((l[0]-width/2)**2 + (l[1]-height)**2)
#         angle = np.arctan2(l[3]-l[1], l[2]-l[0])
#         uvdist.append(dist)
#         uvang.append(angle)

#         l = reallines[i]
#         dist = np.sqrt(l[0]**2 + l[1]**2)
#         angle = np.arctan2(l[3]-l[1], l[2]-l[0])
#         xydist.append(dist)
#         xyang.append(angle)

# plt.scatter(uvdist,uvang)
# plt.legend(["uv"])
# plt.show()
# plt.scatter(xydist,xyang,c="orange")
# plt.legend(["xy"])
# plt.show()

# Find left & right, homography way
left, right = None, None
for i in range(len(linesP)):
    l = linesP[i][0]
    r = reallines[i]
    dist = np.sqrt(r[0]**2 + r[1]**2)
    angle = np.arctan2(r[3]-r[1], r[2]-r[0])
    if r[1] > 0 and abs(angle) < 0.5:
        if left is None:
            left = l
            realleft = r
        elif dist < np.sqrt(realleft[0]**2 + realleft[1]**2):
            left = l
            realleft = r
    if r[1] < 0 and abs(angle) < 0.5:
        if right is None:
            right = l
            realright = r
        elif dist < np.sqrt(realright[0]**2 + realright[1]**2):
            right = l
            realright = r


# # Filter Horizontal Lines
# slope_thresh = np.pi/6
# vertical_lines = []
# for line in linesP:
#     l = line[0]
#     angle = np.arctan2(abs(l[3]-l[1]), l[2]-l[0])
#     # if(slope_thresh < angle < np.pi - slope_thresh):
#     vertical_lines.append(line)

# # Find lanes
# left = None
# right = None
# for i in range(0, len(vertical_lines)):
#     l = vertical_lines[i][0]
#     bot_y = max(l[1], l[3])
#     slope = np.arctan2(l[3]-l[1], l[2]-l[0])
#     if min(l[0], l[2]) <= mid: 
#         if left is None:
#             left = l
#         elif min(l[0], l[2]) > min(left[0], left[2]):
#             left = l
#     else:
#         if right is None:
#             right = l
#         if max(l[0], l[2]) < max(right[0], right[2]):
#             right = l
        
#     cv2.line(color_canny, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
#     # cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

for line in linesP:
    l = line[0]
    cv2.line(color_canny, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

#mark "closest" two lines in green
if left is not None and right is not None:
    cv2.line(color_canny, (left[0], left[1]), (left[2], left[3]), (0,255,0), 3, cv2.LINE_AA)
    cv2.line(color_canny, (right[0], right[1]), (right[2], right[3]), (0,255,0), 3, cv2.LINE_AA)
    # cv2.line(img, (left[0], left[1]), (left[2], left[3]), (0,255,0), 3, cv2.LINE_AA)
    # cv2.line(img, (right[0], right[1]), (right[2], right[3]), (0,255,0), 3, cv2.LINE_AA)

    # Average the two lanes
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

