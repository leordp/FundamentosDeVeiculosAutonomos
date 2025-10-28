import cv2
import numpy as np


def apply_filters(img, parameters):
    p = parameters

    lower = p[0]
    upper = p[1]

    lower_color = np.array(lower)
    upper_color = np.array(upper)

    # HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Color filter
    mask = cv2.inRange(hsv, lower_color, upper_color)
    # img_mask = cv2.bitwise_and(img, img, mask=mask)

    # Erode and dilate
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Canny filter
    # canny = cv2.Canny(mask, 200, 300)

    return mask

def findContours(image,params):
	filtered_image = apply_filters(image,params)
	# Finding Contours 
	# Use a copy of the image e.g. edged.copy() 
	# since findContours alters the image 
	contours, hierarchy = cv2.findContours(filtered_image,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	return contours
