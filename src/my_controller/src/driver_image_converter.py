# Used for converting images from ros to CV
# This is used specifically for driving the robot

import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np

from sensor_msgs.msg import Image

import cv_image_tools
from driving_NN_predict import drivePrediction


PICTURE_SCALE_FACTOR = 0.2


class imageConverter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.new_image)
        self.driver_predictor = drivePrediction()
        self.imgs = []

    # This method gets called whenever there is a new image on the image_raw topic
    def new_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #calculate the scale factor of original dimensions
        width = int(cv_image.shape[1] * PICTURE_SCALE_FACTOR)
        height = int(cv_image.shape[0] * PICTURE_SCALE_FACTOR)

        # resize image
        new_img = cv2.resize(cv_image, (width, height))

        # first or second image
        if len(self.imgs) < 3 :
            self.imgs.append(new_img)
            return
        else:
            self.imgs[0] = self.imgs[1]
            self.imgs[1] = self.imgs[2]
            self.imgs[2] = new_img

        # print(np.shape(self.imgs))

        lin_speed, ang_speed = self.driver_predictor.get_drive_command(self.imgs)

        #### print image for debugging
        # cv2.imshow("1", cv_image)
        # cv2.waitKey(3)

        
