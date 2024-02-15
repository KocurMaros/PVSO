import copy
from ximea import xiapi
import cv2
import numpy as np 
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
k = cv2.waitKey(20)

while k != ord('q'):
    cam.get_image(img)
    image = img.get_image_data_numpy()
    image = cv2.resize(image,(240,240))
    cv2.imshow("test", image)

    if k == ord(' '):
        cv2.imwrite("pictures/test"+str(inc)+".jpg",image)
        inc = inc + 1

    k =cv2.waitKey(20)
    if inc>=4:
        break

picture_list = []
for i in range (4):
    picture_list.append(cv2.imread("pictures/test"+str(i)+".jpg"))

mozaic = cv2.vconcat([cv2.hconcat(picture_list[:2]),cv2.hconcat(picture_list[2:])])

cv2.imshow("mozaic",mozaic)
cv2.waitKey()

height, width, _ = mozaic.shape

roi_height = height // 2 - 1
roi_width = width // 2 - 1

for i in range(roi_height // 2):
    for j in range(i,roi_width-i):
        index_1_i=j
        index_1_j=roi_width-i

        index_2_i=roi_height-i
        index_2_j=roi_width-j

        index_3_i=roi_height-j
        index_3_j=i

        temp = copy.deepcopy(mozaic[i, j])

        mozaic[i,j] = mozaic[index_3_i, index_3_j]
        mozaic[index_3_i, index_3_j] = mozaic[index_2_i, index_2_j]
        mozaic[index_2_i, index_2_j] = mozaic[index_1_i, index_1_j]
        mozaic[index_1_i, index_1_j] = temp



cv2.imshow("mozaic",mozaic)
cv2.waitKey()
       
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