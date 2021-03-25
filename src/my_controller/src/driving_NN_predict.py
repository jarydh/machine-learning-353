import math
import numpy as np
import re
import os
import fnmatch
import cv2

from datetime import datetime
from random import randint

import tensorflow as tf

from tensorflow.keras import models

from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model

from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers

from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend
from matplotlib import pyplot as plt

from driver_pickler import driverPickler 
import driver_controller as dc

import numpy as np

import cv2
import os
import fnmatch


############################## LOAD NN  ###########################
path2 = os.path.dirname(os.path.realpath(__file__)) + "/driving_models/"
path = os.path.dirname(os.path.realpath(__file__)) + "/"


STOP = [dc.LIN_STOP, dc.ANG_STRAIGHT] 
ONE = [1,0,0,0,0,0,0,0,0]

PIVOT_RIGHT = [dc.LIN_STOP, dc.ANG_RIGHT]
TWO = [0,1,0,0,0,0,0,0,0]

PIVOT_LEFT = [dc.LIN_STOP, dc.ANG_LEFT]
THREE = [0,0,1,0,0,0,0,0,0]

DRIVE_SLOW = [dc.LIN_SLOW, dc.ANG_STRAIGHT]
FOUR = [0,0,0,1,0,0,0,0,0]

SLOW_TURN_RIGHT = [dc.LIN_SLOW, dc.ANG_RIGHT]
FIVE = [0,0,0,0,1,0,0,0,0]

SLOW_TURN_LEFT = [dc.LIN_SLOW, dc.ANG_LEFT]
SIX = [0,0,0,0,0,1,0,0,0]

DRIVE_FAST = [dc.LIN_FAST, dc.ANG_STRAIGHT]
SEVEN = [0,0,0,0,0,0,1,0,0]

FAST_TURN_RIGHT = [dc.LIN_FAST, dc.ANG_RIGHT]
EIGHT = [0,0,0,0,0,0,0,1,0]

FAST_TURN_LEFT = [dc.LIN_FAST, dc.ANG_LEFT]
NINE = [0,0,0,0,0,0,0,0,1]




class drivePrediction:

	def __init__(self):
		self.dir = path2
		self.loadNN()

	def loadNN(self):

		drive_NN = models.load_model(path2 + "driver_2021-03-23_22:30:26")
		#drive_NN.summary()

	### Appends three images together - also normalizes data ###
	def appendImages(self, image_triad):

		image_data = image_triad/255

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

		return image_Data

	def guess_to_driving_command(self, one_hot_prediction):
		max_position = np.argmax(one_hot_prediction)

		if max_position == np.argmax(ONE):
			drive_command = STOP
		if max_position == np.argmax(TWO):
			one_hot_data[index] = PIVOT_RIGHT
		if max_position == np.argmax(THREE):
			one_hot_data[index] = PIVOT_LEFT

		if max_position == np.argmax(FOUR):
			one_hot_data[index] = DRIVE_SLOW
		if max_position == np.argmax(FIVE):
			one_hot_data[index] = SLOW_TURN_RIGHT
		if max_position == np.argmax(SIX):
			one_hot_data[index] = SLOW_TURN_LEFT

		if max_position == np.argmax(SEVEN):
			one_hot_data[index] = DRIVE_FAST
		if max_position == np.argmax(EIGHT):
			one_hot_data[index] = FAST_TURN_RIGHT
		if max_position == np.argmax(NINE):
			one_hot_data[index] = FAST_TURN_LEFT

		return drive_command



	def get_drive_command(self, camera_feed):
		three_to_one_image = self.appendImages(camera_feed)

		guess = []
		certainty = 0

		#####blah blah some stuff must go here####

		one_hot_prediction = drive_NN.predict(three_to_one_image)


