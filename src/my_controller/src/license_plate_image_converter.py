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

	#Method everytime there is a new image

	def new_image(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
