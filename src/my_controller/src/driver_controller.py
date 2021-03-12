# This is for sending driving commands to the robot

import rospy
from geometry_msgs.msg import Twist

# Through trial and error I have discovered
# acceleration of 2 will make the car flip
# acceleration of 1.3 will have small jerks but the car stays upright 
# this seems to be independent of the speed of the car
LIN_ACC = 1.3

LIN_STOP = 0
LIN_SLOW = 1
LIN_FAST = 2

# through trial and error, instantaneous angular acceleration does not flip the car
ANG_RIGHT = -1
ANG_STRAIGHT = 0
ANG_LEFT = 1

class driverController:

	# timer is a timer object
	def __init__(self, timer):
		self.vel_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size = 1)
		self.linear_speed = 0
		self.angular_speed = 0
		self.target_linear_speed = 0
		self.target_angular_speed = 0
		self.timer = timer
		self.last_time_ms = timer.get_time_ms()

	def set_linear_speed(self, speed):
		self.target_linear_speed = float(speed)

	def set_angular_speed(self, speed):
		self.target_angular_speed = float(speed)

	def drive(self):
		current_time = self.timer.get_time_ms()
		time_elapsed = current_time - self.last_time_ms
		self.last_time_ms = current_time

		delta_v = LIN_ACC * time_elapsed / 10.0**3
		# print(delta_v)
		# print(self.linear_speed)
		
		# accelerate
		if self.linear_speed < self.target_linear_speed:
			self.linear_speed = self.linear_speed + delta_v
			# if over accelerated
			if self.linear_speed > self.target_linear_speed:
				self.linear_speed = self.target_linear_speed
		# deccelerate
		elif self.linear_speed > self.target_linear_speed:
			self.linear_speed = self.linear_speed - delta_v
			# if over decelerated
			if self.linear_speed < self.target_linear_speed:
				self.linear_speed = self.target_linear_speed
		# speed is correct
		else:
			# do nothing for now
			pass

		# for now, angular acceleration is instantaneous, modify this to add
		# acceleration later if needed
		self.angular_speed = self.target_angular_speed
		print(self.angular_speed)

		# publish vel message
		move = Twist()
		move.linear.x = self.linear_speed
		move.angular.z = self.angular_speed
		self.vel_pub.publish(move)