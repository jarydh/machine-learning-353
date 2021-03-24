#! /usr/bin/env python


from tensorflow.python.keras import layers
from tensorflow.python.keras import models
from tensorflow.python.keras import optimizers

from tensorflow.python.keras.utils import plot_model
from tensorflow.python.keras import backend

import os
import fnmatch

import numpy as np
import cv2

import rospy as rp


# plate dimensions
PLATE_HEIGHT = 298
PLATE_WIDTH = 600


# pixel values for cropping plate images
CHAR_WIDTH = 100
LETTER_TOP = 90
LETTER_BOTTOM = 240

CHAR1_LEFT = 45
CHAR2_LEFT = 145
CHAR3_LEFT = 345
CHAR4_LEFT = 445


rp.init_node('license_main')

DIR = "/home/fizzer/license-plate-NN-353/neural_net/plate_models"

for dirpath, dirs, files in os.walk(DIR):
    # get the plate NN path
    plate_NN_path = os.path.join(dirpath, fnmatch.filter(files, 'plate*')[0])

plate_NN = models.load_model(plate_NN_path)
plate_NN.summary()

print("Loaded plate NN from: " + plate_NN_path)


## load picture
print("Looking for image folders in the following directory:")

img_paths = []

print("Importing plate images from the following directories: test")
# get directories containing images
for dirpath, dirs, files in os.walk("/home/fizzer/license-plate-NN-353/neural_net/test/"):
    for next_img in files:
        img_paths.append(os.path.join(dirpath, next_img))

rawimgset = np.array([cv2.resize(cv2.imread(next_img_path)[47:348,:,:], (PLATE_WIDTH, PLATE_HEIGHT))
                    for next_img_path in img_paths[:]])

print("Loaded {:} images.".format(rawimgset.shape[0]))

IMG_TO_TEST = 0

img = rawimgset[IMG_TO_TEST]

char1 = img[LETTER_TOP:LETTER_BOTTOM, CHAR1_LEFT:CHAR1_LEFT + CHAR_WIDTH, :]
char2 = img[LETTER_TOP:LETTER_BOTTOM, CHAR2_LEFT:CHAR2_LEFT + CHAR_WIDTH, :]
char3 = img[LETTER_TOP:LETTER_BOTTOM, CHAR3_LEFT:CHAR3_LEFT + CHAR_WIDTH, :]
char4 = img[LETTER_TOP:LETTER_BOTTOM, CHAR4_LEFT:CHAR4_LEFT + CHAR_WIDTH, :]

chars = [char1, char2, char3, char4]

guess = ""
certainty = 1
for next_char in chars:
    char_aug = np.expand_dims(next_char, axis=0)

    # # failed bug fix
    # global graph
    # with graph.as_default():
    #     one_hot_prediction = self.plate_NN.predict(char_aug)[0]
    
    one_hot_prediction = plate_NN.predict(char_aug)[0]
    print(one_hot_prediction)
    # certainty, prediction = self.one_hot_to_char(one_hot_prediction)
    # guess = guess + str(prediction)
    # certainty *= certainty
