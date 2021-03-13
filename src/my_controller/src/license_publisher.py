#This is for sending the license plate IDs to the score tracking app

import rospy as rp
from std_msgs.msg import String

class licenseTracker: 

	def __init__(self): 
		self.license_pub = rp.Publisher('/R1/license_plate', String, queue_size = 1)
		self.teamID = 'teamid"
		self.password = 'password'

		self.first_location = 0
		self.last_location = -1

		self.first_message = str('self.teamID, self.password, self.first_location, 0000')
		self.last_message = str('self.teamID, self.password, self.last_location, 0000')

##assuming this function will later be used to pass the license plate ID message to the score tracker 
	#def send_plate_id(self):
		#self.license_pub.publish(message)



