# For converting images from ROS camera to CV form
# Used to read and report lisence plate numbers

import rospy as rp

import numpy as np

import cv2
from cv_bridge import CvBridge
import cv_image_tools

from sensor_msgs.msg import Image

class imageConvert:

	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rp.Subscriber("/R1/pi_camera/image_raw", Image, self.new_image)

	# Method everytime there is a new image
	def new_image(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		# print image for debugging
		cv2.imshow("raw", cv_image)
		cv2.waitKey(3)

		# crop image since we don't need the full picture
		cropped_img = cv_image[360:-1,0:574,:]
		cv2.imshow("cropped", cropped_img)

		# convert to hsv
		hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

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
		percent = 0.9
		masked_sum_thresh = [1 if next_sum > max_sum * percent else 0 for next_sum in masked_sum]

		# gets the first 0 after the first 1
		left_crop = np.argmin(masked_sum_thresh[np.argmax(masked_sum_thresh):]) + np.argmax(masked_sum_thresh) - 1
		# gets the first 1 after the left_crop, offset by 5 just in case there is a little drop
		right_crop = np.argmax(masked_sum_thresh[left_crop + 5:-1]) + (left_crop + 5) - 1
		# crop image based on mask
		cropped_mask = cropped_img[:,left_crop:right_crop,:]
		cropped_mask_hsv = hsv_img[:,left_crop:right_crop,:]

		cv2.imshow("cropped_mask", cropped_mask)


		# set thresholds for grey
		uh = 121
		us = 32
		uv = 119
		lh = 95
		ls = 0
		lv = 89

		lower_hsv = np.array([lh,ls,lv])
		upper_hsv = np.array([uh,us,uv])

		# Threshold the HSV image to get only blue colors
		plate = cv2.inRange(cropped_mask_hsv, lower_hsv, upper_hsv)
		cv2.imshow("plate", plate)

		# MIGHT NOT NEED THIS, also might just want to filter on hsv
		# cropped_mask_grey = cv2.cvtColor(cropped_mask, cv2.COLOR_BGR2GRAY)
		# threshold = 65
		# _, cropped_mask_bin = cv2.threshold(cropped_mask_grey, threshold, 255, cv2.THRESH_BINARY_INV)

		# cv2.imshow("bin", cropped_mask_bin)
