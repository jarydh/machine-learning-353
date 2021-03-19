#! /usr/bin/env python

# On launch, this is the main file that gets run to control the driving.

import rospy

from geometry_msgs.msg import Twist

from driver_image_converter import imageConverter
from timer import simTime
import driver_controller as dc 

# create object to convert images from ROS message to cv2
ic = imageConverter()

# simulation timer
sim_time = simTime()

# initialize ros node
rospy.init_node('driver_main')

# driver object
driver = dc.driverController(sim_time)

# start with 0 velocity
driver.set_linear_speed(dc.LIN_STOP)
driver.set_angular_speed(dc.ANG_STRAIGHT)

# do nothing and loop to keep node active
rate = rospy.Rate(10)
while not rospy.is_shutdown():
	driver.drive()
	rate.sleep()
