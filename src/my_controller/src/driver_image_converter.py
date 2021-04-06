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

# after horizontally summing components, the number of pixels that are lit 
CROSSWALK_INTENSITY_THRESHOLD = 8
# higher number means closer to the robot. this is number of pixels, for scale factor 0.2 this needs to be between 0 and 72
# this will all depend on the stopping distance of the robot
CROSSWALK_DISTANCE_THRESHOLD = 20

MOTION_DIFF_TRHESHOLD = 50

# how many pixels (vertically) needed
MOTION_PIXEL_THRESHOLD = 2

CROSSWALK_DETECTION_COOLDOWN = 3 # seconds
SPEED_THROUGH_CROSSWALK = 0.2

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
        self.is_at_crosswalk = False
        self.motion_detected = True
        self.current_img = None
        self.previous_img = None
        self.croswalk_detection_enabled = True

    def reset_crosswalk_detection(self, timer_event):
        self.croswalk_detection_enabled = True
        self.crosswalk_cooldown_timer.shutdown()

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
        new_img = cv2.resize(cv_image, (width, height))

        self.previous_img = self.current_img
        self.current_img = new_img

        # check conditions
        # TODO: Check if should move to inner loop
        # check if at a crosswalk
        if self.croswalk_detection_enabled:
            self.is_at_crosswalk = self.check_for_crosswalk(new_img)

        if self.is_at_crosswalk:
            self.croswalk_detection_enabled = False
            self.motion_detected = self.check_for_motion(self.current_img, self.previous_img)
            status = "Waiting for motion to stop"
            lin_speed = 0
            ang_speed = 0
            # can resume normal driving
            if not self.motion_detected:
                self.is_at_crosswalk = False
                self.crosswalk_cooldown_timer = rospy.Timer(rospy.Duration(CROSSWALK_DETECTION_COOLDOWN), self.reset_crosswalk_detection)

        else:
            status = "Driving Outer Loop"

            self.imgs = new_img

            #check for NN
            if self.driver_predictor.isLoaded:
                lin_speed, ang_speed = self.driver_predictor.get_drive_command(self.imgs)
            else:
                return

            ######## FROM 3-FRAME NN #########################
            # # first or second image
            # if len(self.imgs) < 3 :
            #     self.imgs.append(new_img)
            #     return
            # # all other cases
            # else:
            #     self.imgs[0] = self.imgs[1]
            #     self.imgs[1] = self.imgs[2]
            #     self.imgs[2] = new_img

            # # check if NN is loaded before using
            # if self.driver_predictor.isLoaded:
            #     lin_speed, ang_speed = self.driver_predictor.get_drive_command(self.imgs)
            # else:
            #     return


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
        show_text = 'Lin Speed: ' + str(lin_speed) + '  Ang Speed: ' + str(ang_speed)
        status_text = "Status: " + status
        cv2.putText(show_img, show_text, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)
        bottomLeftCornerOfText = (10,80)
        cv2.putText(show_img, status_text, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)
        cv2.imshow("driving_prediction", show_img)
        cv2.waitKey(3)

        self.driver.set_linear_speed(lin_speed)
        self.driver.set_angular_speed(ang_speed)

        
    # returns true if the car should stop at a crosswalk
    def check_for_crosswalk(self, img):
        # crop image
        cropped_img = img[np.shape(img)[0]/2:,:,:]

        # # Show hsv mask for debugging
        # cv2.imshow("cropped", cropped_img)

        # convert to hsv
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        # set thresholds for red
        uh = 3
        us = 255
        uv = 255
        lh = 0
        ls = 87
        lv = 110

        lower_hsv = np.array([lh,ls,lv])
        upper_hsv = np.array([uh,us,uv])

        # Threshold the HSV image to get only blue colors
        masked_img = cv2.inRange(hsv_img, lower_hsv, upper_hsv)

        # # Show hsv mask for debugging
        # cv2.imshow("masked", masked_img)

        # horizontally sum components, normallize
        masked_sum = np.sum(masked_img, axis=1)/255

        # threshold to percent of max
        max_sum = np.max(masked_sum)

        if max_sum < CROSSWALK_INTENSITY_THRESHOLD:
            # no crosswalk detected
            return False

        max_location = np.argmax(masked_sum)
        if max_location > CROSSWALK_DISTANCE_THRESHOLD:
            # crosswalk close enough
            return True
        else:
            # crosswalk not close enough
            return False


    # check for motion in the current frame vs the previous one
    def check_for_motion(self, current_img, previous_img):
        # TODO: crop these down
        current_img_crop = current_img
        previous_img_crop = previous_img

        current_img_grey = cv2.cvtColor(current_img_crop, cv2.COLOR_BGR2GRAY)
        previous_img_grey = cv2.cvtColor(previous_img_crop, cv2.COLOR_BGR2GRAY)


        difference = cv2.absdiff(current_img_grey, previous_img_grey) 
        _, difference_thresh = cv2.threshold(difference, MOTION_DIFF_TRHESHOLD, 255, cv2.THRESH_BINARY)

        # vertially sum components, normallize
        masked_sum = np.sum(difference_thresh, axis=0)/255

        # cv2.imshow("motion", difference_thresh)
        # print(np.max(masked_sum))

        return np.max(masked_sum) > MOTION_PIXEL_THRESHOLD
