# Used for converting images from ros to cv

import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np

from sensor_msgs.msg import Image


class imageConverter:

	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.new_image)

	# This method gets called whenever there is a new image on the image_raw topic
	def new_image(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv2.imshow(cv_image)