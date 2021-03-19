#This is for sending the license plate IDs to the score tracking app

import rospy as rp
from std_msgs.msg import String

class licenseTracker: 

	def __init__(self): 
		self.license_pub = rp.Publisher('/license_plate', String, queue_size = 1)
		self.teamID = 'teamid'
		self.password = 'password'

	##assuming this function will later be used to pass the license plate ID message to the score tracker 
	def sendPlateID(self, location, plate):
		
		location = str(location)
		plate = str(plate)

		message = str(self.teamID+', '+self.password+', '+location+', '+plate)

		self.license_pub.publish(message)

