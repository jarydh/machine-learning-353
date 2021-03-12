# Object to get simulation time for timestamping stuff
import rospy

from rosgraph_msgs.msg import Clock

class simTime:

	def __init__(self):
		self.clock_sub = rospy.Subscriber("/clock", Clock, self.next_time)
		self.current_time_ms = 0

	# This method gets called whenever there is a new image on the image_raw topic
	def next_time(self, data):
		self.current_time_ms = data.clock.secs * 10**3 + data.clock.nsecs / 10**6
		# # print time for debugging
		# print(self.current_time_ns)

	def get_time_ms(self):
		return self.current_time_ms
