import math
import numpy as np
import re
import os
import fnmatch
import cv2

from datetime import datetime
from random import randint

from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers

from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend
from matplotlib import pyplot as plt

from driver_pickler import driverPickler 
import driver_controller as dc



############################## LOAD NN  ###########################
path2 = os.path.dirname(os.path.realpath(__file__)) + "/driving_models/"

drive_NN = models.load_model(path2 + "driver_2021-03-23_22:30:26")
drive_NN.summary()


path = os.path.dirname(os.path.realpath(__file__)) + "/"

images = []

pickle = driverPickler()

for dirpath, dirs, files in os.walk(path + "driving_data/"):  
	for filename in fnmatch.filter(files, '*_imgs.pickle'): 
		timestamp = filename[0:19]
		in_pickle = pickle.load_pickle(timestamp)

		images.append(in_pickle[0])

group_image = []

count = 0

for i in range(0,len(images)):
	for j in range(2,len(images[i])):

		image_set = []

		image_set.append(images[i][j-2])
		image_set.append(images[i][j-1])
		image_set.append(images[i][j])

		for k in range(3):
			img = image_set[k]
			img = img.astype(float)
			img_arr = np.asarray(img)
			image_set[k] = img_arr

		group_image.append(image_set)

		count+=1 

image_data = np.array(group_image)
image_data = image_data/255

spot = 0
dim1 = np.shape(image_data)[0]
dim2 = np.shape(image_data)[2]
dim3 = np.shape(image_data)[3]*3
dim4 = np.shape(image_data)[4]

image_Data = np.empty((dim2, dim3, dim4))
image_Data = np.expand_dims(image_Data, axis =0)
image_Data = np.repeat(image_Data, dim1, axis=0)

for group in image_data:
	arr1 = np.array(group[0])
	arr2 = np.array(group[1])
	arr3 = np.array(group[2])

	combined_arr = np.hstack((arr1, arr2, arr3))

	image_Data[spot] = combined_arr

	spot+=1

test_image = []
num = randint(0, 500)
test_image.append(image_Data[num])
num = randint(0, 500)
test_image.append(image_Data[num])
num = randint(0, 500)
test_image.append(image_Data[num])
num = randint(0, 500)
test_image.append(image_Data[num])
num = randint(0, 500)
test_image.append(image_Data[num])

test_image = np.array(test_image)



# print(test_image)
# print(np.shape(test_image))

# cv2.imshow("1", test_image)
# cv2.waitKey(0)


one_hot_prediction = drive_NN.predict(test_image)

print(one_hot_prediction)


#     # returns a list with the 4 characters in the image
#     def break_to_chars(self, img):
#         char1 = img[LETTER_TOP:LETTER_BOTTOM, CHAR1_LEFT:CHAR1_LEFT + CHAR_WIDTH, :]
#         char2 = img[LETTER_TOP:LETTER_BOTTOM, CHAR2_LEFT:CHAR2_LEFT + CHAR_WIDTH, :]
#         char3 = img[LETTER_TOP:LETTER_BOTTOM, CHAR3_LEFT:CHAR3_LEFT + CHAR_WIDTH, :]
#         char4 = img[LETTER_TOP:LETTER_BOTTOM, CHAR4_LEFT:CHAR4_LEFT + CHAR_WIDTH, :]
        
#         # show first char for debugging
#         # raw_input()
#         # cv2.imshow("2", char2)
#         # cv2.imshow("3", char3)
#         # cv2.imshow("4", char4)
#         # cv2.waitKey(3)
#         # raw_input()

#         return [char1, char2, char3, char4]

#     # returns tuple of certainty, and prediction
#     def one_hot_to_char(one_hot):
#       max_index = np.argmax(one_hot)
#       if max_index < 26:
#         char = chr(max_index + 65)
#       else:
#         char = chr(max_index + 22)

#       return (one_hot[max_index], char)


#     def guess_plate(self, plate_img):
#         plate_img = cv2.resize(plate_img, (PLATE_WIDTH, PLATE_HEIGHT))
#         chars = self.break_to_chars(plate_img)

#         # # check inputs
#         # cv2.imshow("char1", chars[0])
#         # cv2.imshow("char2", chars[1])
#         # cv2.imshow("char3", chars[2])
#         # cv2.imshow("char4", chars[3])
#         guess = ""
#         certainty = 1
#         for next_char in chars:
#             char_aug = np.expand_dims(next_char, axis=0)

#             # # failed bug fix
#             # global graph
#             # with graph.as_default():
#             #     one_hot_prediction = self.plate_NN.predict(char_aug)[0]
            
#             one_hot_prediction = self.plate_NN.predict(char_aug)[0]
#             certainty, prediction = self.one_hot_to_char(one_hot_prediction)
#             guess = guess + str(prediction)
#             certainty *= certainty

#         return (certainty, prediction)
