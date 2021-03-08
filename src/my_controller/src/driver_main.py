#! /usr/bin/env python

# On launch, this is the main file that gets run to control the driving.

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


from image_converter import imageConverter


ic = imageConverter()
rospy.init_node('driver_main')

rate = rospy.Rate(10)

while not rospy.is_shutdown():
	rate.sleep()
