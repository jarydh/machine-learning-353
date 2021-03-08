# This is for sending driving commands to the robot

import rospy

from geometry_msgs.msg import Twist

class driverController:

	def __init__(self):
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

# TODO implement driving functions

# for reference 
	# def move(self, angular_z):
	# 	move_duration = rospy.Duration(0.1)

	# 	print(angular_z)

	# 	move = Twist()
	# 	if abs(angular_z) < 0.20:
	# 		move.linear.x = 0.2
	# 	else:
	# 		move.linear.x = 0
	# 	move.angular.z = angular_z
	# 	self.vel_pub.publish(move)