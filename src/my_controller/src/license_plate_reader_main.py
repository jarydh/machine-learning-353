#! /usr/bin/env python

# On launch, this is the main file that gets run to control the license plate detection.

#SET TEAM ID AND PASSWORD IN LICENSE_PUBLISHER FILE

import rospy as rp
from license_plate_image_converter import imageConvert

import license_publisher as lpub

def stop(timer_event):
    guess_publisher.sendStop()

guess_publisher = lpub.licenseTracker()

#imager converter object
ic = imageConvert(guess_publisher)

#initialize node
rp.init_node('license_main')

# sleep for 4 seconds to let everything load
rp.sleep(4.)

# rp.sleep(1.)

# send the start command
guess_publisher.sendStart()

# send the stop command after 4 minutes
stop_timer = rp.Timer(rp.Duration(4.*60), stop, oneshot=True)

# FOR TIME TRIALS: send the stop command after 30 seconds
# stop_timer = rp.Timer(rp.Duration(30), stop)

rate = rp.Rate(10)
while not rp.is_shutdown():
	rate.sleep()
