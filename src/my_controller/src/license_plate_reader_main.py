#! /usr/bin/env python

# On launch, this is the main file that gets run to control the license plate detection.

#SET TEAM ID AND PASSWORD IN LICENSE_PUBLISHER FILE

import rospy as rp
from license_plate_image_converter import imageConvert
import license_publisher as lpub

#imager converter object
ic = imageConvert()

#initialize node
rp.init_node('license_main')

#license plate object
plate  = lpub.licenseTracker()


## Just here for testing - working fine. 0 and -1 are registering as not a number for plate locations so I'm assuming that is correct
#location = -1
#plate_value = 'A4R8'

rate = rp.Rate(10)
while not rp.is_shutdown():
	#plate.sendPlateID(location, plate_value)
	rate.sleep()
