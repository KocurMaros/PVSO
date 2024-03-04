import numpy as np
import cv2 as cv
import glob
from ximea import xiapi
def nothing(x):
    pass
img = cv.imread('opencv-logo-white.png', cv.IMREAD_GRAYSCALE)
cv.namedWindow('image')
cv.createTrackbar('P1','image',50,100,nothing)
cv.setTrackbarMin('P1', 'image', 1) 
cv.createTrackbar('P2','image',30,100,nothing)
cv.setTrackbarMin('P2', 'image', 1) 
cv.createTrackbar('minR','image',5,100,nothing)
cv.createTrackbar('maxR','image',30,100,nothing)
cv.createTrackbar('blur','image',1,100,nothing)
use_ximea = True
gaussian_amount = 10

# termination criteria
if use_ximea == True:
    cam = xiapi.Camera()

    print('Opening first camera...')
    cam.open_device()

    cam.set_exposure(50000)
    cam.set_param("imgdataformat","XI_RGB32")
    cam.set_param("auto_wb",1)

    print('Exposure was set to %i us' %cam.get_exposure())
    img = xiapi.Image()
    print('Starting data acquisition...')
    cam.start_acquisition()
    

    inc = 0
    k = cv.waitKey(20)

    while k != ord('q'):
        cam.get_image(img)
        # make img grayscale
        image = img.get_image_data_numpy()
        image= cv.resize(image,(400,300))
        gimg = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        if k == ord('s'):
            cv.imwrite("pictures/original.jpg",image)
            cv.imwrite("pictures/grayscale.jpg",gimg)
        # gimg = cv.medianBlur(gimg,5)

        gaussian_amount = cv.getTrackbarPos('blur','image')
        for i in range(gaussian_amount):
            gimg = cv.GaussianBlur(gimg,(5,5),0)

        if k == ord('s'):
            cv.imwrite("pictures/blur.jpg",gimg)
        
        cimg = image

        canny = cv.Canny(gimg, threshold1=cv.getTrackbarPos('P1','image'), threshold2=cv.getTrackbarPos('P1','image')*2)
        cv.imshow("canny", canny)

        circles = cv.HoughCircles(gimg,cv.HOUGH_GRADIENT,1,minDist=20,
                                param1=cv.getTrackbarPos('P1','image'),
                                param2=cv.getTrackbarPos('P2','image'),
                                minRadius=cv.getTrackbarPos('minR','image'),
                                maxRadius=cv.getTrackbarPos('maxR','image'))

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
        # draw the outer circle
                cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
                cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
        cv.imshow("image", cimg)
        cv.imshow("blur", gimg)
        if k == ord('s'):
            cv.imwrite("pictures/hough.jpg",image)
            cv.imwrite("pictures/canny.jpg",canny)
        
        k = cv.waitKey(5)
