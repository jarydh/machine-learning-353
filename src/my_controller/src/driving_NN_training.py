#! /usr/bin/env python


####IMPORTS######

import math
import numpy as np
import re
import os
import fnmatch
import cv2

from datetime import datetime

from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers

from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend
from matplotlib import pyplot as plt

from driver_pickler import driverPickler 
import driver_controller as dc


####IMPORTING DATA########################################

path = os.path.dirname(os.path.realpath(__file__)) + "/"

images = []
directions = []

pickle = driverPickler()

for dirpath, dirs, files in os.walk(path + "driving_data/"):  
	for filename in fnmatch.filter(files, '*_imgs.pickle'): 
		timestamp = filename[0:19]
		#file_path = os.path.join(dirpath, filename)
		#print(timestamp)
		#import_ = pickle.load_pickle(timestamp)
		in_pickle = pickle.load_pickle(timestamp)

		images.append(in_pickle[0])
		directions.append(in_pickle[1])

# cv2.imshow("1", images[0][1])
# cv2.waitKey(0)
	
# plt.show()	
# print(directions[0])
# print(np.shape(images[0][0]))
# print(len(images))
# print(len(images[0]))
# print(len(images[1]))

##The following 2 for loops group 3 images with their 'leading' label

group_image = []
group_directions = []

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

for i in range(0,len(directions)):
	for j in range(2,len(directions[i])):
		group_directions.append(directions[i][j])


# print(len(group_directions))
# print(len(group_image))
# print(image_set[0])
# print(img)


NUMBER_OF_LABELS = 9
CONFIDENCE_THRESHOLD = 0.01


##ONE HOT ENCODING####
#labels: [LIN_STOP, ANG_STRAIGHT], [LIN_STOP, ANG_RIGHT], [LIN_STOP, ANG_LEFT]
#        [LIN_SLOW, ANG_STRAIGHT], [LIN_SLOW, aNG_RIGHT], [LIN_SLOW, ANG_LEFT]
#		 [LIN_FAST, ANG_STRAIGHT], [LIN_FAST, ANG_RIGHT], [LIN-FAST, ANG_LEFT]


#[LIN_STOP, ANG_STRAIGHT] 
ONE = [1,0,0,0,0,0,0,0,0]
#[LIN_STOP, ANG_RIGHT]
TWO = [0,1,0,0,0,0,0,0,0]
#[LIN_STOP, ANG_LEFT]
THREE = [0,0,1,0,0,0,0,0,0]
#[LIN_SLOW, ANG_STRAIGHT]
FOUR = [0,0,0,1,0,0,0,0,0]
#[LIN_SLOW, ANG_RIGHT]
FIVE = [0,0,0,0,1,0,0,0,0]
#[LIN_SLOW, ANG_LEFT]
SIX = [0,0,0,0,0,1,0,0,0]
#[LIN_FAST, ANG_STRAIGHT]
SEVEN = [0,0,0,0,0,0,1,0,0]
#[LIN_FAST, ANG_RIGHT]
EIGHT = [0,0,0,0,0,0,0,1,0]
#[LIN-FAST, ANG_LEFT]
NINE = [0,0,0,0,0,0,0,0,1]

one_hot_data = np.empty((len(group_directions),9))
index = 0

for label in group_directions:

	if label == (dc.LIN_STOP, dc.ANG_STRAIGHT):
		one_hot_data[index] = ONE
	if label == (dc.LIN_STOP, dc.ANG_RIGHT):
		one_hot_data[index] = TWO
	if label == (dc.LIN_STOP, dc.ANG_LEFT):
		one_hot_data[index] = THREE
	if label == (dc.LIN_SLOW, dc.ANG_STRAIGHT):
		one_hot_data[index] = FOUR
	if label == (dc.LIN_SLOW, dc.ANG_RIGHT):
		one_hot_data[index] = FIVE
	if label == (dc.LIN_SLOW, dc.ANG_LEFT):
		one_hot_data[index] = SIX
	if label == (dc.LIN_FAST, dc.ANG_STRAIGHT):
		one_hot_data[index] = SEVEN
	if label == (dc.LIN_FAST, dc.ANG_RIGHT):
		one_hot_data[index] = EIGHT
	if label == (dc.LIN_FAST, dc.ANG_LEFT):
		one_hot_data[index] = NINE

	index += 1

# print(np.shape(one_hot_data))
# print(one_hot_data[8])

###NORMALIZING IMAGES####

image_data = np.array(group_image)
#print(image_data)
image_data = image_data/255
#print(image_data[4][0])

# print(np.shape(image_data))

##########################APPEND 3 IMAGES TOGETHER######################
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

# print(combined_arr)
# print(np.shape(combined_arr))
# print(np.shape(image_Data))
# print(image_Data[8])

######################SHUFFLE DATA######################
all_data = []

for img_idx, next_img in enumerate(image_Data):
	 	next_label = one_hot_data[img_idx]
		data_pair = np.array([next_img, next_label])
		all_data.append(data_pair)

#print(all_data[300])
#print(np.shape(all_data))

np.random.shuffle(all_data)


X_dataset = np.array([data[0] for data in all_data[:]])
Y_dataset = np.array([data[1] for data in all_data])

# print(np.shape(X_dataset))
# print(np.shape(Y_dataset))


VALIDATION_SPLIT = 0.2
# print("Total examples: {:f}\nTraining examples: {:f}\nTest examples: {:f}".
#       format(one_hot_data.shape[0],
#              math.ceil(one_hot_data.shape[0] * (1-VALIDATION_SPLIT)),
#              math.floor(one_hot_data.shape[0] * VALIDATION_SPLIT)))


####FUNCTION FOR REINITIALIZING MODEL PARAMETERS#############

def reset_weights(model):
	session = backend.get_session()
	for layer in model.layers:
		if hasattr(layer, 'kernel_initializer'):
			layer.kernel.initializer.run(session=session)

########NN MODEL DEFINITION############

conv_model = models.Sequential()
conv_model.add(layers.Conv2D(32, (3, 3), activation='relu',
                             input_shape=np.shape(X_dataset[0])))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(64, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(64, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
# conv_model.add(layers.Conv2D(128, (3, 3), activation='relu'))
# conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Flatten())
conv_model.add(layers.Dropout(0.5))
conv_model.add(layers.Dense(512, activation='relu'))
conv_model.add(layers.Dense(9, activation='softmax'))

#conv_model.summary()


################SETTING UP FOR TRAINING###############

LEARNING_RATE = 5e-4
conv_model.compile(loss='categorical_crossentropy', optimizer=optimizers.RMSprop(lr=LEARNING_RATE),metrics=['acc'])


reset_weights(conv_model)

history_conv = conv_model.fit(X_dataset, Y_dataset, validation_split=VALIDATION_SPLIT, epochs=20, batch_size=16)


save_dirpath =  path + "driving_models/driver_" +  datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
models.save_model(conv_model, save_dirpath)
print("Saved to: " + save_dirpath)

