# Object to get simulation time for timestamping stuff
import rospy

from rosgraph_msgs.msg import Clock

class simTime:

	def __init__(self):
		self.clock_sub = rospy.Subscriber("/clock", Clock, self.next_time)

	# This method gets called whenever there is a new image on the image_raw topic
	def next_time(self, data):
		self.current_time_ns = data.clock.nsecs
		self.current_time_sec = data.clock.secs
		# # print time for debugging
		# print(self.current_time_ns)

	def get_time_ns(self):
		return self.current_time_ns

	def get_time_sec(self):		
		return self.current_time_sec