import numpy as np
import cv2 as cv
def nothing(x):
    pass
img = cv.imread('opencv-logo-white.png', cv.IMREAD_GRAYSCALE)
cv.namedWindow('image')
cv.createTrackbar('P1','image',50,100,nothing)
cv.setTrackbarMin('P1', 'image', 1) 
cv.createTrackbar('P2','image',30,100,nothing)
cv.setTrackbarMin('P2', 'image', 1) 
cv.createTrackbar('minR','image',5,100,nothing)
cv.createTrackbar('maxR','image',100,100,nothing)
assert img is not None, "file could not be read, check with os.path.exists()"
img = cv.medianBlur(img,5)

k=cv.waitKey(5)

while k!=ord('q'):
    cimg = cv.cvtColor(img,cv.COLOR_GRAY2BGR)
    circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,20,
                                param1=cv.getTrackbarPos('P1','image'),
                                param2=cv.getTrackbarPos('P2','image'),
                                minRadius=cv.getTrackbarPos('minR','image'),
                                maxRadius=cv.getTrackbarPos('maxR','image'))
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
    cv.imshow('image',cimg)
    k = cv.waitKey(5)

cv.destroyAllWindows()