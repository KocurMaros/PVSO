import cv2 as cv
import numpy as np
import hough_transform as ht
import kanye as west

def nothing():
    pass

image = cv.imread('test_image.png')
# get size of the image
height, width, _ = image.shape
cv.namedWindow('image', cv.WINDOW_NORMAL)
cv.resizeWindow('image', width, height)

cv.createTrackbar('C1','image',300,1000,nothing)
cv.setTrackbarMin('C1', 'image', 1)
cv.createTrackbar('C2','image',700,1000,nothing)
cv.setTrackbarMin('C2', 'image', 1)

k = cv.waitKey(5)
while (k != ord):
    image = cv.imread('test_image.png')
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    # edges = cv.Canny(gray, 50, 150, apertureSize=3)
    edges = west.canny_edge_detection(gray, low_threshold=cv.getTrackbarPos('C1','image'), high_threshold=cv.getTrackbarPos('C2','image'))
    # accumulator = ht.hough_transform(edges)
    # print(accumulator)
    # lines = ht.detect_lines(accumulator, threshold=300)
    # image = ht.draw_lines(image, lines)
    cv.imshow('image', edges)
    # accumulator = cv.HoughLines(edges, 1, np.pi/180, threshold=200)
    # lines = accumulator[:, 0, :]
    # image = ht.draw_lines(image, lines)
    # cv.imshow('image2', image)
    k = cv.waitKey(5)
cv.destroyAllWindows()


