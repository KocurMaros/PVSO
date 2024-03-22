import cv2 as cv
import math
import numpy as np
import hough_transform as ht
import kanye as west
import time

def nothing(x):
    pass

image = cv.imread('test_image.png')
# get size of the image
height, width, _ = image.shape
# cv.namedWindow('image', cv.WINDOW_NORMAL)
# cv.resizeWindow('image', width, height)
# cv.createTrackbar('C1','image',10,100,nothing)
# cv.setTrackbarMin('C1', 'image', 1)
# cv.createTrackbar('C2','image',10,100,nothing)
# cv.setTrackbarMin('C2', 'image', 1)

image = cv.imread('test_image.png')
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
canny = west.canny_edge_detection(gray, low_threshold=40, high_threshold=980)
canny = cv.Mat(canny)
cv.imshow('image', canny)
lines = ht.hough_transform(canny, threshold=150)
# print(lines)
if lines is not None:
    for rho, theta in lines:
        a = np.cos(np.deg2rad(theta))
        b = np.sin(np.deg2rad(theta))
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        pt1 =  (x1, y1)
        pt2 = (x2, y2)
        cv.line(image, pt1, pt2, (0,0,255), 3, cv.LINE_AA)

cv.imshow('image2', image)
cv.waitKey(0)
cv.destroyAllWindows()


