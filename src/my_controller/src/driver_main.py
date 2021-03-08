#! /usr/bin/env python

# On launch, this is the main file that gets run to control the driving.

import rospy

from geometry_msgs.msg import Twist

from driver_image_converter import imageConverter
from timer import simTime
from driver_controller import driverController

# create object to convert images from ROS message to cv2
ic = imageConverter()

# simulation timer
sim_time = simTime()

# initialize ros node
rospy.init_node('driver_main')


# do nothing and loop to keep node active
rate = rospy.Rate(10)
while not rospy.is_shutdown():
	rate.sleep()
