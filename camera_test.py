import copy
from ximea import xiapi
import cv2
import numpy as np 
### runn this command first echo 0|sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb  ###

# create instance for first connected camera
# cam = xiapi.Camera()

# # start communication
# # to open specific device, use:
# # cam.open_device_by_SN('41305651')
# # (open by serial number)
# print('Opening first camera...')
# cam.open_device()

# # settings
# cam.set_exposure(50000)
# cam.set_param("imgdataformat","XI_RGB32")
# cam.set_param("auto_wb",1)

# print('Exposure was set to %i us' %cam.get_exposure())

# # create instance of Image to store image data and metadata
# img = xiapi.Image()

# # start data acquisitionq
# print('Starting data acquisition...')
# cam.start_acquisition()

# inc = 0
# k = cv2.waitKey(20)

# while k != ord('q'):
#     cam.get_image(img)
#     image = img.get_image_data_numpy()
#     image = cv2.resize(image,(240,240))
#     cv2.imshow("test", image)

#     if k == ord(' '):
#         cv2.imwrite("pictures/test"+str(inc)+".jpg",image)
#         inc = inc + 1

#     k =cv2.waitKey(20)
#     if inc>=4:
#         break

#load pictures to mozaic
picture_list = []
for i in range (4):
    picture_list.append(cv2.imread("pictures/test"+str(i)+".jpg"))

mozaic = cv2.vconcat([cv2.hconcat(picture_list[:2]),cv2.hconcat(picture_list[2:])])

cv2.imwrite("pictures/mozaic.jpg",mozaic)
cv2.imshow("mozaic",mozaic)
cv2.waitKey()

#kernel mask
height, width = mozaic.shape[:2]
quarter_height, quarter_width = height // 2, width // 2 

kernel = np.ones((5,5),np.float32)/25

quarter = mozaic[:quarter_height, :quarter_width]

filtered_quarter = cv2.filter2D(quarter, -1, kernel)

mozaic[:quarter_height, :quarter_width] = filtered_quarter

cv2.imshow("mozaic",mozaic)
cv2.waitKey()

#rotate picture 
for i in range(quarter_height // 2):
    for j in range(i+quarter_width,width-i-1):
        index_1_i=j-quarter_width
        index_1_j=width-1-i

        index_2_i=quarter_height-1-i
        index_2_j=width-1-j+quarter_width

        index_3_i=quarter_height-1-j+quarter_width
        index_3_j=i+quarter_width

        temp = copy.deepcopy(mozaic[i, j])

        mozaic[i,j] = mozaic[index_3_i, index_3_j]
        mozaic[index_3_i, index_3_j] = mozaic[index_2_i, index_2_j]
        mozaic[index_2_i, index_2_j] = mozaic[index_1_i, index_1_j]
        mozaic[index_1_i, index_1_j] = temp



cv2.imshow("mozaic",mozaic)
cv2.waitKey()

# showing only red channel
b,g,r = cv2.split(mozaic)

zeros = np.zeros(mozaic.shape[:2], dtype="uint8")

for i in range(quarter_height,height):
    for j in range(quarter_width+1):
        mozaic[i, j] = [zeros[i, j], zeros[i, j], r[i, j]]

# Display the image
cv2.imshow("mozaic",mozaic)
cv2.waitKey()


# stop data acquisition
# print('Stopping acquisition...')
# cam.stop_acquisition()

# # stop communication
# cam.close_device()

# print("Data type of the image: ", mozaic.dtype)
# print("Dimensions of the image: ", mozaic.shape)
# print("Size of the image: ", mozaic.size)
# print('Done.')