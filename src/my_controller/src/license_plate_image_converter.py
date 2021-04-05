# For converting images from ROS camera to CV form
# Used to read and report lisence plate numbers

import rospy as rp

import numpy as np

import cv2
from cv_bridge import CvBridge
import cv_image_tools

from sensor_msgs.msg import Image

from plate_stall_NN_guesser import plateStallGuesser


# number of pixels that meet HSV threshold for it to consider a plate
PLATE_GREY_TRHESHOLD = 200

class imageConvert:

    def __init__(self, guess_publisher):
        self.bridge = CvBridge()
        self.image_sub = rp.Subscriber("/R1/pi_camera/image_raw", Image, self.new_image)
        self.outer_loop()
        self.ps_guesser = plateStallGuesser()
        self.guess_publisher = guess_publisher

    # call this if driving on the inner loop
    def inner_loop(self):
        self.on_outer_loop = False

    # call this if driving on the outer loop
    def outer_loop(self):
        self.on_outer_loop = True

    # Method everytime there is a new image
    def new_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        plate, stall = self.get_images(cv_image)
        if plate is not None:
            plate_certainty, plate_prediction = self.ps_guesser.guess_plate(plate)

            # for debugging
            plate = cv2.resize(plate, (600, 298))
            font                   = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = (10,40)
            fontScale              = 1.2
            fontColor              = (0, 0,255)
            lineType               = 3
            text = 'Guess: ' + str(plate_prediction)

            cv2.putText(plate, text, 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                lineType)
            cv2.imshow("plate_guess", plate)
            cv2.waitKey(3)
        else:
            return

        if stall is not None:
            stall_certainty, stall_prediction = self.ps_guesser.guess_stall(stall)

            # for debugging
            stall = cv2.resize(stall, (260, 320))
            font                   = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = (10,40)
            fontScale              = 1.2
            fontColor              = (0, 0,255)
            lineType               = 3
            text = 'Guess: ' + str(stall_prediction)

            cv2.putText(stall, text, 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                lineType)
            cv2.imshow("stall_guess", stall)
            cv2.waitKey(3)
        else:
            return
        
        # publish
        if plate_prediction is not None and stall_prediction is not None:
            self.guess_publisher.sendPlateID(stall_prediction, plate_prediction)



    # returns the images for the license plate and the parking stall number
    def get_images(self, cv_image):
        # print image for debugging
        # cv2.imshow("raw", cv_image)
        # cv2.waitKey(3)

        # crop image to bottom left corner since we don't need the full picture
        y_min = 360 # min y coordinate
        x_max = 574 # max x coordinate

        if self.on_outer_loop:
            cropped_img = cv_image[y_min:, :x_max, :]
        else:
            cropped_img = cv_image[y_min:, -x_max:, :]

        # cv2.imshow("cropped", cropped_img)

        # convert to hsv
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        # get coordinates for cropping around blue
        left_crop, right_crop = self.apply_blue_hsv_mask(hsv_img)

        # crop image based on mask
        cropped_mask = cropped_img[:,left_crop:right_crop,:]
        cropped_mask_hsv = hsv_img[:,left_crop:right_crop,:]

        # cv2.imshow("cropped_mask", cropped_mask)


        # blue section does not meet threshold to get meaningful data
        blue_width_min = 40
        blue_width_max = 300

        if np.shape(cropped_mask)[1] < blue_width_min or np.shape(cropped_mask)[1] > blue_width_max:
            return (None, None)


        # get coordinates for transforming plate
        plate_pts = self.apply_grey_hsv_mask(cropped_mask_hsv)

        # no plate found
        if plate_pts is None:
            return(None, None)

        # transform plate
        plate_transform = self.perpective_transform(plate_pts, cropped_mask)
        
        
        if plate_transform is None:
            # No license plate found
            return(None, None)

        # check to make sure we actually have a plate by filtering for the grey colour
        # thresholds for grey
        uh = 121
        us = 32
        uv = 193
        lh = 95
        ls = 0
        lv = 89

        lower_hsv = np.array([lh,ls,lv])
        upper_hsv = np.array([uh,us,uv])

        # convert to hsv
        plate_transform_hsv = cv2.cvtColor(plate_transform, cv2.COLOR_BGR2HSV)
        # Threshold the HSV image to get only grey colors
        plate_transform_hsv = cv2.inRange(plate_transform_hsv, lower_hsv, upper_hsv)
        plate_grey_sum = np.sum(plate_transform_hsv) / 255
        print(plate_grey_sum)

        if plate_grey_sum < PLATE_GREY_TRHESHOLD:
            return (None, None)

        tl = plate_pts[0]
        tr = plate_pts[1]
        br = plate_pts[2]
        bl = plate_pts[3]
        width = tr[0]

        # shift coordinates up to find stall number
        left_scale = bl[1] - tl[1]
        right_scale = br[1] - tr[1]

        # how many times the height of the plate should we move up
        top_scale = 2.2
        bot_scale = 1.3

        stall_pts = np.array([
            [0, tl[1] - left_scale * top_scale],
            [width, tr[1] - right_scale * top_scale],
            [width, br[1] - right_scale * bot_scale],
            [0, bl[1]] - left_scale * bot_scale], dtype = "float32")

        # transform stall
        stall_transform = self.perpective_transform(stall_pts, cropped_mask)

        # resize the image for consistency, then crop around the number
        stall_transform = cv2.resize(stall_transform, (160, 95))
        stall_transform = stall_transform[15:95,95:,:]

        # cv2.imshow("plate", plate_transform)
        # cv2.imshow("stall", stall_transform)


        return(plate_transform, stall_transform)


    # takes HSV image, returns left and right coordinates to vertically crop around the blue color
    def apply_blue_hsv_mask(self, hsv_img):
        # set thresholds for blue
        uh = 130
        us = 255
        uv = 255
        lh = 110
        ls = 50
        lv = 50

        lower_hsv = np.array([lh,ls,lv])
        upper_hsv = np.array([uh,us,uv])

        # Threshold the HSV image to get only blue colors
        masked_img = cv2.inRange(hsv_img, lower_hsv, upper_hsv)

        # Show hsv mask for debugging
        # cv2.imshow("masked", masked_img)

        # vertically sum components
        masked_sum = np.sum(masked_img, axis=0)

        # threshold to percent of max
        max_sum = np.max(masked_sum)
        percent = 0.7
        masked_sum_thresh = np.array([1 if next_sum > max_sum * percent else 0 for next_sum in masked_sum])

        # # for debugging
        # percent_max = np.array([next_sum / float(max_sum) for next_sum in masked_sum])
        # print(percent_max)

        # gets the first 0 after the first 1
        left_crop = np.argmin(masked_sum_thresh[np.argmax(masked_sum_thresh):]) + np.argmax(masked_sum_thresh) - 1
        # gets the first 1 after the left_crop, offset by 5 just in case there is a little drop
        right_crop = np.argmax(masked_sum_thresh[left_crop + 5:-1]) + (left_crop + 5) - 1

        return (left_crop, right_crop)

    # takes HSV image, returns np array with coordinates marking the corners of the grey plate color
    def apply_grey_hsv_mask(self, cropped_mask_hsv):
        # set thresholds for grey
        uh = 121
        us = 32
        uv = 193
        lh = 95
        ls = 0
        lv = 89

        lower_hsv = np.array([lh,ls,lv])
        upper_hsv = np.array([uh,us,uv])

        # Threshold the HSV image to get only grey colors
        plate_masked = cv2.inRange(cropped_mask_hsv, lower_hsv, upper_hsv)

        # # Show hsv mask for debugging
        # cv2.imshow("plate_mask", plate_masked)

        # horizontally sum components on edge pixels, include range to account for missing points
        num_pixels = 10
        left_sum = np.sum(plate_masked[:,:num_pixels], axis=1) / 255
        right_sum = np.sum(plate_masked[:,-num_pixels:], axis=1) / 255


        # number of pixels present
        pixel_threshold = 3

        left_sum = np.array([1 if next_sum >= pixel_threshold else 0 for next_sum in left_sum])
        right_sum = np.array([1 if next_sum >= pixel_threshold else 0 for next_sum in right_sum])

        # no plate at all
        if np.sum(left_sum + right_sum) == 0:
            return None

        # ignores certain number of pixels on the top and botom
        top_ignore = 20
        bot_ignore = 150

        # gets the first 1
        left_top = np.argmax(left_sum[top_ignore:]) + top_ignore
        # gets the last 1
        left_bottom = np.size(left_sum) - 1 - np.argmax(np.flip(left_sum[:-bot_ignore])) - bot_ignore

        # gets the first 1
        right_top = np.argmax(right_sum[top_ignore:]) + top_ignore
        # gets the last 1
        right_bottom = np.size(right_sum) - 1 - np.argmax(np.flip(right_sum[:-bot_ignore])) - bot_ignore

        width = np.shape(cropped_mask_hsv)[1]
        
        plate_pts = np.array([
            [0, left_top],
            [width - 1, right_top],
            [width - 1, right_bottom],
            [0, left_bottom]], dtype = "float32")

        # print(plate_pts)

        return plate_pts

    def perpective_transform(self, pts, cropped_mask):
        # from: https://www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/
        # compute the width of the new image, which will be the
        # maximum distance between bottom-right and bottom-left
        # x-coordiates or the top-right and top-left x-coordinates
        tl = pts[0]
        tr = pts[1]
        br = pts[2]
        bl = pts[3]

        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))
        # compute the height of the new image, which will be the
        # maximum distance between the top-right and bottom-right
        # y-coordinates or the top-left and bottom-left y-coordinates
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))
        # now that we have the dimensions of the new image, construct
        # the set of destination points to obtain a "birds eye view",
        # (i.e. top-down view) of the image, again specifying points
        # in the top-left, top-right, bottom-right, and bottom-left
        # order
        destination_pts = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype = "float32")

        # # draw points on image for debugging
        # points_draw = cropped_mask
        # for i in range(4):
        #   points_draw = cv2.circle(points_draw, (pts[i][0], pts[i][1]), 2, (0, 255, 0), 2)
        # cv2.imshow("plate", points_draw)

        transform_matrix = cv2.getPerspectiveTransform(pts, destination_pts)

        try:
            transform_matrix = cv2.getPerspectiveTransform(pts, destination_pts)
            transformed = cv2.warpPerspective(cropped_mask, transform_matrix, (maxWidth, maxHeight))

            # if the plate detected is less than this number of pixels, assume we haven't found a plate
            plate_width_thresh = 15
            if np.shape(transformed)[1] < plate_width_thresh:
                # no plate detected
                transformed = None
        except:
            transformed = None
            print("Failed")

        return transformed