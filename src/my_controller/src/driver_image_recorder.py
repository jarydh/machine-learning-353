# used for recording driving images to train NN

import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime

import driver_pickler as dp

from sensor_msgs.msg import Image

import cv_image_tools

# scale to this factor
PICTURE_SCALE_FACTOR = 0.2


class driverImageRecorder:

    def __init__(self, driver_object):
        self.driver = driver_object
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.new_image)
        self.is_recording = False
        self.current_frame = None
        self.recorded_frames = []
        self.recorded_speeds = []   
        self.current_recording_timestamp = None
        self.driver_pickler = dp.driverPickler()       

    # This method gets called whenever there is a new image on the image_raw topic
    def new_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        #calculate the scale factor of original dimensions
        width = int(cv_image.shape[1] * PICTURE_SCALE_FACTOR)
        height = int(cv_image.shape[0] * PICTURE_SCALE_FACTOR)

        # resize image
        self.current_frame = cv2.resize(cv_image, (width, height))
        # # print image for debugging
        # cv2.imshow("raw", cv_image)
        # cv2.imshow("scaled", self.current_frame)
        # cv2.waitKey(3)

    def toggle_is_recording(self):
        if self.is_recording:
            print("RECORDING OFF")
            self.is_recording = False
            # save current recording in pickle file
            self.driver_pickler.save_pickle(self.current_recording_timestamp, self.recorded_frames, self.recorded_speeds)
            self.recorded_frames = []
            self.recorded_speeds = []   
        else:
            print("RECORDING ON")
            self.current_recording_timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
            self.is_recording = True

    def capture_frame(self):
        if self.is_recording:
            frame = self.current_frame
            target_speeds = (self.driver.get_linear_speed(), self.driver.get_angular_speed())

            self.recorded_frames.append(frame)
            self.recorded_speeds.append(target_speeds)

