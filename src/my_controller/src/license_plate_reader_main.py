#! /usr/bin/env python

# On launch, this is the main file that gets run to control the license plate detection.

import rospy as rp
from license_plate_image_converter import imageConvert
import license_publisher as lpub

#imager converter object
ic = imageConvert()

#initialize node
rp.init_node('license_main')

#license plate object
plate  = lpub.licenseTracker()


rate = rp.Rate(10)
while not rp.is_shutdown():
	rate.sleep()

#TODO: finish main code  - right now I am not sure how/when the liscense 
#plate IDs will be passed to the score tracker 
