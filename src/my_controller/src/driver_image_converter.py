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

    def __init__(self, driver_controller):
        self.bridge = CvBridge()
        self.imgs = []
        
        self.driver = driver_controller
        # start with 0 velocity
        self.driver.set_linear_speed(0)
        self.driver.set_angular_speed(0)
        self.driver_predictor = drivePrediction()

        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.new_image)

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
        if self.driver_predictor.isLoaded:
            lin_speed, ang_speed = self.driver_predictor.get_drive_command(self.imgs)
        else:
            return


        # for debugging
        #calculate the scale factor of original dimensions
        show_width = int(cv_image.shape[1] * 0.5)
        show_height = int(cv_image.shape[0] * 0.5)

        # resize image
        show_img = cv2.resize(cv_image, (show_width, show_height))

        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10,40)
        fontScale              = 1.1
        fontColor              = (0, 0,255)
        lineType               = 3
        text = 'Lin Speed: ' + str(lin_speed) + '  Ang Speed: ' + str(ang_speed)

        cv2.putText(show_img, text, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)
        cv2.imshow("driving_prediction", show_img)
        cv2.waitKey(3)

        self.driver.set_linear_speed(lin_speed)
        self.driver.set_angular_speed(ang_speed)

        
