from turtle import color
import cv2
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template, color_to_detect = "orange"):
    """
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
			(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
    if color_to_detect == "blue":
        color_min = np.array([90, 50, 70])
        color_max = np.array([128, 255, 255])
    elif color_to_detect == "orange":
        color_min = np.array([5, 50, 125])
        color_max = np.array([15, 255, 255])
    hsv_image = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    # image_print(hsv_image)
    mask = cv2.inRange(hsv_image, color_min, color_max)
	# image_print(mask)
    erosion_kernel = np.ones((15,15), np.uint8)
    img_erosion = cv2.erode(mask, erosion_kernel, iterations=1)
    # image_print(img_erosion)
    dilation_kernel = np.ones((20,20), np.uint8)
    img_dilation = cv2.dilate(img_erosion, dilation_kernel, iterations=1)
    # image_print(img_dilation)
    segmented_img = cv2.bitwise_and(img, img, mask=img_dilation)
    # image_print(segmented_img)
    contours= cv2.findContours(img_dilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    x,y,w,h = cv2.boundingRect(contours[0])
    #out = cv2.rectangle(img, (x,y), (x+w, y+h),(0,255,0),2)
	#image_print(out)e
    bounding_box = ((x,y),(x+w,y+h))

    # Return bounding box
    return bounding_box


if __name__ == "__main__":
	img = cv2.imread('./test_images_cone/test14.jpg')
	cd_color_segmentation(img, None)