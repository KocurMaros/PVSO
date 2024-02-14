from ximea import xiapi
import cv2
import numpy as np 
from time import sleep
### runn this command first echo 0|sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb  ###

# create instance for first connected camera
cam = xiapi.Camera()

# start communication
# to open specific device, use:
# cam.open_device_by_SN('41305651')
# (open by serial number)
print('Opening first camera...')
cam.open_device()

# settings
cam.set_exposure(50000)
cam.set_param("imgdataformat","XI_RGB32")
cam.set_param("auto_wb",1)

print('Exposure was set to %i us' %cam.get_exposure())

# create instance of Image to store image data and metadata
img = xiapi.Image()

# start data acquisitionq
print('Starting data acquisition...')
cam.start_acquisition()


inc = 0
while cv2.waitKey() != ord('q'):

    ke = cv2.waitKey()   
    # if ke == ord(' '):
    cam.get_image(img)
    image = img.get_image_data_numpy()
    image = cv2.resize(image,(240,240))
    cv2.imshow("test", image)
    cv2.imwrite("test"+str(inc)+".jpg", image)
    inc = inc + 1
    if inc == 4 :
        break

grid_image = images
fig, axis = plt.subplots(2,2)
axis[0,0].imshow(images[0])
axis[0,1].imshow(images[1])
axis[1,0].imshow(images[2])
axis[1,1].imshow(images[3])



img1 = cv2.imread('test0.jpg') 
img2 = cv2.imread('test1.jpg') 
img3 = cv2.imread('test2.jpg') 
img4 = cv2.imread('test3.jpg') 
  
# concatenate image Horizontally 
Hori1 = np.concatenate((img1, img2), axis=1)    
Hori2 = np.concatenate((img3, img4), axis=1) 
Verti = np.concatenate((Hori1, Hori2), axis=0) 
# cv2.imshow('HORIZONTAL', Hori) 
cv2.imwrite("mozaic.jpg", Verti)
# sleep(10)
cv2.waitKey(0)   
    # for i in range(4):
        # cam.get_image(img)
        # image = img.get_image_data_numpy()
        # image = cv2.resize(image,(240,240))

        # cv2.imshow("test", image)
    # if keyboard.is_pressed('space'):          
# for i in range(10):
#     #get data and pass them from camera to img
#     cam.get_image(img)
#     image = img.get_image_data_numpy()
#     cv2.imshow("test", image)
#     cv2.waitKey()
#     #get raw data from camera
#     #for Python2.x function returns string
#     #for Python3.x function returns bytes
#     data_raw = img.get_image_data_raw()
#
#     #transform data to list
#     data = list(data_raw)
#
#     #print image data and metadata
#     print('Image number: ' + str(i))
#     print('Image width (pixels):  ' + str(img.width))
#     print('Image height (pixels): ' + str(img.height))
#     print('First 10 pixels: ' + str(data[:10]))
#     print('\n')

# stop data acquisition
print('Stopping acquisition...')
cam.stop_acquisition()

# stop communication
cam.close_device()

print('Done.')