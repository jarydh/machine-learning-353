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
DRIVE_NN_PATH = os.path.dirname(os.path.realpath(__file__)) + "/driving_models/"

sess1 = tf.Session()    
graph1 = tf.get_default_graph()
set_session(sess1)


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

    # type is a string, either "outer" or "inner"
    def __init__(self, type):
        self.dir = DRIVE_NN_PATH
        self.isLoaded = False
        self.loadNN(type)

    def loadNN(self, type):
        # # look for NN files
        for dirpath, dirs, files in os.walk(self.dir):
            # get the plate and stall NN path
            drive_NN_path = os.path.join(dirpath, fnmatch.filter(files, '*' + str(type) + '_driver*')[0])

        self.drive_NN = models.load_model(drive_NN_path)
        print("Loaded driving NN from: " + drive_NN_path)
        self.isLoaded = True

        # drive_NN.summary()

    ### Appends three images together - also normalizes data ###
    ##### Not used anymroe due to new NN ####
    def appendImages(self, image_triad):
        image_triad = np.array(image_triad)
        image_data = image_triad/255.

        arr1 = np.array(image_data[0])
        arr2 = np.array(image_data[1])
        arr3 = np.array(image_data[2])

        combined_arr = np.hstack((arr1, arr2, arr3))

        return combined_arr

    def guess_to_driving_command(self, one_hot_prediction):
        max_position = np.argmax(one_hot_prediction)

        if max_position == np.argmax(ONE):
            drive_command = STOP
        if max_position == np.argmax(TWO):
            drive_command = PIVOT_RIGHT
        if max_position == np.argmax(THREE):
            drive_command = PIVOT_LEFT

        if max_position == np.argmax(FOUR):
            drive_command = DRIVE_SLOW
        if max_position == np.argmax(FIVE):
            drive_command = SLOW_TURN_RIGHT
        if max_position == np.argmax(SIX):
            drive_command = SLOW_TURN_LEFT

        if max_position == np.argmax(SEVEN):
            drive_command = DRIVE_FAST
        if max_position == np.argmax(EIGHT):
            drive_command = FAST_TURN_RIGHT
        if max_position == np.argmax(NINE):
            drive_command = FAST_TURN_LEFT

        return drive_command



    def get_drive_command(self, camera_feed):
        #three_to_one_image = self.appendImages(camera_feed)

        image = camera_feed
        image = np.array(image)
        image = image.astype(np.float32)
        norm_image = image/255
        img_aug = np.expand_dims(norm_image, axis=0)


        global sess1
        global graph1
        with graph1.as_default():
                set_session(sess1)
                NN_prediction = self.drive_NN.predict(img_aug)

        drive_command = self.guess_to_driving_command(NN_prediction)

        return drive_command