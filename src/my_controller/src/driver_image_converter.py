# Used for converting images from ros to CV
# This is used specifically for driving the robot

import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String


import cv_image_tools
from driving_NN_predict import drivePrediction
import manual_driver as md
import driver_controller as dc

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

POSSIBLE_TRANSITION_STALLS = [1, 3, 4, 6]

INNER_LOOP_SLOW_SCALE = 0.4

TRUCK_DETECTION_TIMER = 1

class imageConverter:

    def __init__(self, driver_controller):
        self.bridge = CvBridge()

        # driver NN
        self.outer_driver_NN = drivePrediction("outer")
        self.inner_driver_NN = drivePrediction("inner")
        # start with the outer loop
        self.driver_predictor = self.outer_driver_NN

        # image feed
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.new_image)
        self.imgs = []
        self.current_img = None
        self.previous_img = None

        # driver controller
        self.driver = driver_controller
        # start with 0 velocity
        self.driver.set_linear_speed(0)
        self.driver.set_angular_speed(0)

        # manual driver
        self.manual_driver = md.manualDriver()

        # to communicate with the plate NN
        self.drive_status_pub = rospy.Publisher("/driver_status", String, queue_size = 1)

        # driver status variables
        self.is_active = False
        self.do_left_turn = True
        self.is_waiting_for_motion = False
        self.motion_detected = True
        self.croswalk_detection_enabled = True
        self.is_on_outer = True
        self.is_transitioning_loops = False
        self.pause_before_turn = False
        self.motion_detection_count = 0

        # plate tracking variables
        self.stall_guess_sub = rospy.Subscriber("/stall_guess", String, self.new_plate_guess)
        self.stall_guesses = set()
        self.most_recent_stall_guess = 0

    # whenever a new plate guess is made
    def new_plate_guess(self, data):
        # special number for starting
        if int(data.data) == 0:
            self.is_active = True
            self.driver.is_active = True
        # special value for stopping
        elif int(data.data) == -1:
            self.is_active = False
            # self.driver.is_active = False
        else:
            self.stall_guesses.add(data.data)
            self.most_recent_stall_guess = int(data.data)

    def reset_crosswalk_detection(self, timer_event):
        self.croswalk_detection_enabled = True

    def set_transition_pause(self, timer_event):
        if self.is_waiting_for_motion == True:
            self.pause_before_turn = True
        
    def reset_restrict_motion(self, timer_event):
        self.motion_detection_count = 1

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

        # check if should move to inner loop
        if len(self.stall_guesses) >= 6 and self.is_on_outer and self.most_recent_stall_guess in POSSIBLE_TRANSITION_STALLS:
            self.croswalk_detection_enabled = False
            self.is_transitioning_loops = True
        # check if at a crosswalk
        elif self.croswalk_detection_enabled:
            self.is_waiting_for_motion = self.check_for_crosswalk(new_img)


        # get the driving commands
        if not self.is_active:
            status = "Waiting to start"
            lin_speed = 0
            ang_speed = 0
        elif self.do_left_turn:
            if self.pause_before_turn:
                status = "Manual left turn with pause"
                lin_speed, ang_speed, is_done = self.manual_driver.pause_left_turn()
            else:
                status = "Manual left turn"            
                lin_speed, ang_speed, is_done = self.manual_driver.left_turn()
            if is_done:
                self.do_left_turn = False
        elif self.is_waiting_for_motion:
            self.croswalk_detection_enabled = False
            self.motion_detected = self.check_for_motion(self.current_img, self.previous_img)
            status = "Waiting for motion to stop"
            lin_speed = 0
            ang_speed = 0
            # can resume normal driving
            if not self.motion_detected:
                self.is_waiting_for_motion = False
                self.motion_detection_count += 1
                if self.motion_detection_count == 2:
                    rospy.Timer(rospy.Duration(5), self.reset_restrict_motion, oneshot=True)
                # if on the outer loop, means we stopped for a crosswalk
                if self.is_on_outer:
                    self.crosswalk_cooldown_timer = rospy.Timer(rospy.Duration(CROSSWALK_DETECTION_COOLDOWN), self.reset_crosswalk_detection, oneshot=True)
                # if not, then it means we stopped for the truck
                else:
                    try:
                        self.truck_waiting_timer.shutdown()
                    except:
                        pass
                    self.do_left_turn = True
        elif self.is_transitioning_loops:
            status = "Transitioning between loops"
            # drive between loops 
            lin_speed, ang_speed, is_done = self.manual_driver.loop_transition()
            if is_done:
                self.is_transitioning_loops = False
                self.is_on_outer = False
                self.driver_predictor = self.inner_driver_NN
                self.is_waiting_for_motion = True
                self.drive_status_pub.publish("Inner")
                self.truck_waiting_timer = rospy.Timer(rospy.Duration(TRUCK_DETECTION_TIMER), self.set_transition_pause, oneshot=True)
        else:
            if self.is_on_outer:
                status = "Driving Outer Loop"
            else:
                status = "Driving Inner Loop"

            self.imgs = new_img

            #check for NN
            if self.driver_predictor.isLoaded:
                lin_speed, ang_speed = self.driver_predictor.get_drive_command(self.imgs)
            else:
                return

            if self.is_on_outer == False:
                lin_speed = lin_speed * INNER_LOOP_SLOW_SCALE
                ang_speed = ang_speed * INNER_LOOP_SLOW_SCALE

            if self.is_on_outer == True and self.motion_detection_count >= 2 and lin_speed == dc.LIN_FAST:
                lin_speed = dc.LIN_SLOW
                status = status + ", restricting speed."

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

        return np.max(masked_sum) >= MOTION_PIXEL_THRESHOLD
