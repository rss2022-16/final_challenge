import cv2
import numpy as np

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
    height = 376
    width = 672
    mid = int(width/2)
    kernel_size = 7
    lower_white = np.array([0,0,168])
    upper_white = np.array([172,111,255])
    crop_cutoff = 2/5

    ## Lane detecting
    angle_cutoff = np.pi/2 ## NOTE - IN X,Y COORDS meaning 0deg = vertical!

    
    def __init__(self):

        cv2.namedWindow("out", cv2.WINDOW_NORMAL)

        #Initialize data into a homography matrix
        np_pts_ground = np.array(self.PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * self.METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])
        np_pts_image = np.array(self.PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])
        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

    def imgtoline(self, img):
        """
        Given a ros image type, return detected line in x,y local robot coordinates. 
        First point (x1,y1)
        Returns: Line segment ((x1,y1),(x2,y2))
        """
        img = cv2.resize(img, (self.width, self.height))

        # Blur
        blurred = cv2.GaussianBlur(img,(self.kernel_size, self.kernel_size),0)

        # Color Filtering
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_white, self.upper_white)

        # Edge Detection
        canny = cv2.Canny(mask, 50, 200, None, 3)
        color_canny = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)

        # Crop
        y0, yf = int(self.crop_cutoff * self.height), self.height
        cropped = np.zeros(canny.shape,np.uint8)
        cropped[y0:yf,:] = canny[y0:yf,:]

        # Hough Transform
        linesP = cv2.HoughLinesP(cropped, 1, np.pi / 180, 50, None, 80, 10)
        for line in linesP:
            l = line[0]
            cv2.line(color_canny, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

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

        # Find lanes
        left, right = None, None
        for i in range(len(linesP)):
            r = reallines[i]
            l = linesP[i][0]

            dist = np.sqrt(r[0]**2 + r[1]**2)
            angle = np.arctan2(r[3]-r[1], r[2]-r[0])

            # Left lane?
            if l[1] < self.mid and abs(angle) < self.angle_cutoff:
                if left is None:
                    left = r
                    ul = l
                elif dist < np.sqrt(left[0]**2 + left[1]**2):
                    left = r
                    ul = l

            # Right lane?
            if l[1] > self.mid and abs(angle) < self.angle_cutoff:
                if right is None:
                    right = r
                    ur = l
                elif dist < np.sqrt(right[0]**2 + right[1]**2):
                    right = r
                    ur = l

        # print(ur[0], ur[1], ur[2], ur[3])
        # print(right[0], right[1], right[2], right[3])
        # print(ul[0], ul[1], ul[2], ul[3])
        # print(left[0], left[1], left[2], left[3])

        # If we detected lanes
        if left is not None and right is not None:

            # cv2.line(color_canny, (ul[0], ul[1]), (ul[2], ul[3]), (0,255,0), 3, cv2.LINE_AA)
            # cv2.line(color_canny, (ur[0], ur[1]), (ur[2], ur[3]), (0,255,0), 3, cv2.LINE_AA)

            # Average the two lanes
            bot_x = left[0]/2 + right[0]/2
            bot_y = left[1]/2 + right[1]/2
            top_x = left[2]/2 + right[2]/2
            top_y = left[3]/2 + right[3]/2

            # ## Temp
            # bot_u = int(ul[0]/2 + ur[0]/2)
            # bot_v = int(ul[1]/2 + ur[1]/2)
            # top_u = int(ul[2]/2 + ur[2]/2)
            # top_v = int(ul[3]/2 + ur[3]/2)
            # cv2.line(color_canny, (bot_u, bot_v), (top_u, top_v), (255,0,0), 3, cv2.LINE_AA)
            # cv2.imshow('out', color_canny)
            # cv2.waitKey(0)

            return ((bot_x, bot_y),(top_x, top_y))

        else:

            return (None,None)
        

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
    line = detector.imgtoline(img)
    print(line)