# used for guessing text using the NN

import tensorflow as tf

from tensorflow.keras import models

from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model

import numpy as np

import cv2
import os
import fnmatch


NN_DIR = os.path.dirname(os.path.realpath(__file__)) + "/plate_stall_NN/"


# pixel values for cropping plate images
CHAR_WIDTH = 100
LETTER_TOP = 90
LETTER_BOTTOM = 240

CHAR1_LEFT = 45
CHAR2_LEFT = 145
CHAR3_LEFT = 345
CHAR4_LEFT = 445

# plate dimensions
PLATE_HEIGHT = 298
PLATE_WIDTH = 600

# stall dimensions
STALL_HEIGHT = 80
STALL_WIDTH = 65

sess1 = tf.Session()    
graph1 = tf.get_default_graph()
set_session(sess1)

class plateStallGuesser:
    def __init__(self):
        self.dir = NN_DIR
        self.loadNN()

    def loadNN(self):
        # # look for NN files
        for dirpath, dirs, files in os.walk(self.dir):
            # get the plate and stall NN path
            plate_NN_path = os.path.join(dirpath, fnmatch.filter(files, 'plate*')[0])
            stall_NN_path = os.path.join(dirpath, fnmatch.filter(files, 'stall*')[0])

        self.plate_NN = models.load_model(plate_NN_path)
        self.stall_NN = models.load_model(stall_NN_path)

        print("Loaded plate NN from: " + plate_NN_path)
        print("Loaded stall NN from: " + stall_NN_path)


    # returns a list with the 4 characters in the image
    def break_to_chars(self, img):
        char1 = img[LETTER_TOP:LETTER_BOTTOM, CHAR1_LEFT:CHAR1_LEFT + CHAR_WIDTH, :]
        char2 = img[LETTER_TOP:LETTER_BOTTOM, CHAR2_LEFT:CHAR2_LEFT + CHAR_WIDTH, :]
        char3 = img[LETTER_TOP:LETTER_BOTTOM, CHAR3_LEFT:CHAR3_LEFT + CHAR_WIDTH, :]
        char4 = img[LETTER_TOP:LETTER_BOTTOM, CHAR4_LEFT:CHAR4_LEFT + CHAR_WIDTH, :]
        
        # show first char for debugging
        # raw_input()
        # cv2.imshow("2", char2)
        # cv2.imshow("3", char3)
        # cv2.imshow("4", char4)
        # cv2.waitKey(3)
        # raw_input()

        return [char1, char2, char3, char4]

    # returns tuple of certainty, and prediction
    def result_to_char(self, one_hot):
      max_index = np.argmax(one_hot)
      if max_index < 26:
        char = chr(max_index + 65)
      else:
        char = chr(max_index + 22)

      return (one_hot[max_index], char)


    def guess_plate(self, plate_img):
        plate_img = cv2.resize(plate_img, (PLATE_WIDTH, PLATE_HEIGHT))
        chars = self.break_to_chars(plate_img)

        # # check inputs
        # cv2.imshow("char1", chars[0])
        # cv2.imshow("char2", chars[1])
        # cv2.imshow("char3", chars[2])
        # cv2.imshow("char4", chars[3])
        guess = ""
        certainty = 1
        for next_char in chars:
            char_aug = np.expand_dims(next_char, axis=0)

            global sess1
            global graph1
            with graph1.as_default():
                set_session(sess1)
                NN_prediction = self.plate_NN.predict(char_aug)[0]
            
            certainty, prediction = self.result_to_char(NN_prediction)
            guess += str(prediction)
            certainty *= certainty


        return (certainty, guess)


    def guess_stall(self, stall_img):
        stall_img = cv2.resize(stall_img, (STALL_WIDTH, STALL_HEIGHT))
        guess = ""
        certainty = 1
        stall_aug = np.expand_dims(stall_img, axis=0)

        global sess1
        global graph1
        with graph1.as_default():
            set_session(sess1)
            NN_prediction = self.stall_NN.predict(stall_aug)[0]

        guess = np.argmax(NN_prediction)
        certainty = NN_prediction[guess]

        # since index 0 corresponds to stall 1
        guess += 1
        return (certainty, guess)