import numpy as np
import cv2 as cv
import glob
from ximea import xiapi

if 1 == 0:
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
        image = img.get_image_data_numpy()
        image = cv.resize(image,(300,300))
        cv.imshow("test", image)

        if k == ord(' '):
            cv.imwrite("pictures/calibration"+str(inc)+".jpg",image)
            inc = inc + 1

        k =cv.waitKey(20)
        if inc>=10:
            break

    print('Stopping acquisition...')
    cam.stop_acquisition()

    # stop communication
    cam.close_device()

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((5*6,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:5].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('pictures/calibration0.jpg')
inc = 0 
for fname in images:
    print(fname)
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    # cv.imshow('img', gray)
    # cv.waitKey()
    ret, corners = cv.findChessboardCorners(gray, (6,5), None)
    # print(ret)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (6,5), corners2, ret)
        cv.imshow('img', img)
        # cv.imwrite("pictures/out/out"+str(inc)+".jpg",img)
        _, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        h,  w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        dst = cv.undistort(img, mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        cv.imwrite('pictures/out/cal'+str(inc)+'.png', dst)
        print("Camera Matrix : \n", mtx)
        print("Dist : \n", dist)
        print("------------------------")
        cv.waitKey(500)
    inc = inc + 1 
cv.destroyAllWindows()

img = cv.imread('pictures/calibration0.jpg')
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('pictures/undistort/calibresult.png', dst)