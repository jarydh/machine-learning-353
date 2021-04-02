# This is for sending driving commands to the robot

import rospy
from geometry_msgs.msg import Twist

# Through trial and error I have discovered
# acceleration of 5 will make the car jerk a lot
# acceleration of 3 will make it jerk a little
# acceleration of 2 is optimal
# this seems to be independent of the speed of the car
LIN_ACC = 2

# the main issue with jerking while accelerating is if the car tries to 
# deccelerate while it is jerking, it will flip

# Through trial and error I have discovered
# deceleration of 2 will make the car flip
# deceleration of 1.7 will have the rear wheels come off the ground
# deceleration of 1.5 will have small jerks but the car stays upright 
# this seems to be independent of the speed of the car
LIN_DEC = 1.5

LIN_STOP = 0
# note: do not try taking corners at speeds > 1, it will roll the car
LIN_SLOW = 0.1
LIN_FAST = 0.2

# through trial and error, discovered that instantaneous angular acceleration 
# does not flip the car

# turn radius: the corners have a radius so the angular velocity should be set
# to match v = rw so that the car can take the turns on slow speed
RADIUS = 0.32 # approx
ANG_RIGHT = -1 * LIN_SLOW / RADIUS
ANG_STRAIGHT = 0
ANG_LEFT = LIN_SLOW / RADIUS

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

	def get_linear_speed(self):
		return self.target_linear_speed

	def get_angular_speed(self):
		return self.target_angular_speed

	def drive(self):
		current_time = self.timer.get_time_ms()
		time_elapsed = current_time - self.last_time_ms
		self.last_time_ms = current_time

		# print(self.target_linear_speed)
		# print(self.linear_speed)
		
		# accelerate
		if self.linear_speed < self.target_linear_speed:
			delta_v = LIN_ACC * time_elapsed / 10.0**3
			self.linear_speed = self.linear_speed + delta_v
			# if over accelerated
			if self.linear_speed > self.target_linear_speed:
				self.linear_speed = self.target_linear_speed
		# deccelerate
		elif self.linear_speed > self.target_linear_speed:
			delta_v = LIN_DEC * time_elapsed / 10.0**3
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
		# print(self.angular_speed)

		# publish vel message
		move = Twist()
		move.linear.x = self.linear_speed
		move.angular.z = self.angular_speed
		self.vel_pub.publish(move)