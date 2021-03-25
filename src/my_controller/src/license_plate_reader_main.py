#! /usr/bin/env python

# On launch, this is the main file that gets run to control the license plate detection.

#SET TEAM ID AND PASSWORD IN LICENSE_PUBLISHER FILE

import rospy as rp
from license_plate_image_converter import imageConvert

import license_publisher as lpub

guess_publisher = lpub.licenseTracker()

#imager converter object
ic = imageConvert(guess_publisher)

#initialize node
rp.init_node('license_main')

# sleep for 3 seconds to let everything load
rp.sleep(3.)

# send the start command
guess_publisher.sendStart()

# send the stop command after 4 minutes
rp.Timer(rp.Duration(4.*60), guess_publisher.sendStop())

## Just here for testing - working fine. 0 and -1 are registering as not a number for plate locations so I'm assuming that is correct
#location = -1
#plate_value = 'A4R8'

rate = rp.Rate(10)
while not rp.is_shutdown():
	rate.sleep()
