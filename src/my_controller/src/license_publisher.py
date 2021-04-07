#This is for sending the license plate IDs to the score tracking app

import rospy as rp
from std_msgs.msg import String
import numpy as np

PLATE_TIMEOUT = 1
PLATE_MINIMUM = 3

class licenseTracker: 

	def __init__(self): 
		self.license_pub = rp.Publisher('/license_plate', String, queue_size = 2)
		self.stall_guess_pub = rp.Publisher('/stall_guess', String, queue_size = 1)
		self.teamID = 'teamid'
		self.password = 'password'
		self.current_plate_predictions = []
		self.num_predictions = 0
		self.stall_guesses = set()
		self.is_outer = True

	# return None if the guess should be ignored, otherwise return corrected guess
	def correct_plate(self, location_guess, plate_guess):
		if plate_guess[0].isalpha() and plate_guess[1].isalpha() and plate_guess[2].isdigit() and plate_guess[3].isdigit():
			if self.is_outer == True and location_guess > 6:
				return None
			elif self.is_outer == False and location_guess <=6:
				return None
			else:
				return plate_guess
		else:
			# TODO: try and correct if the guess is close (1s to I, etc)
			# This is almost better to do nothing, because we reject bad readings
			# instead of trying to guess on them
			return None

	def sendPlateID(self, location, plate):
		self.num_predictions += 1

		# check if the prediction is valid
		corrected_plate = self.correct_plate(location, plate)

		if corrected_plate is not None:
			# add prediction to current predictions
			self.current_plate_predictions.append((location, plate))


		# try to shutdown the plate timer if it exists
		try:
			self.plate_timer.shutdown()
		except:
			pass

		# start the countdown timer
		self.plate_timer = rp.Timer(rp.Duration(PLATE_TIMEOUT), self.publish_prediction, oneshot=True)

	# for now, this just returns the most recent guess but we could add functionality to 
	# take the most common guess or something if that is better
	def get_guess(self, plate_predictions):
		location = plate_predictions[-1][0]
		plate = plate_predictions[-1][1]
		return (location, plate) 

	def publish_prediction(self, timer_event):
		# check if there are actually any plate guesses and if there are enough plates detected
		if len(self.current_plate_predictions) > 0  and self.num_predictions >= PLATE_MINIMUM:
			location, plate = self.get_guess(self.current_plate_predictions)
			self.ROS_publish(location, plate)

			self.stall_guesses.add(location)
			print("Guessed plate: " + str(len(self.stall_guesses)) + "/8")
			if len(self.stall_guesses) >= 8:
				self.sendStop()
		
		self.current_plate_predictions = []
		self.num_predictions = 0


	# send the stop command
	def sendStart(self):
		self.ROS_publish(0, "AA00")

	# send the stop command
	def sendStop(self):
		self.ROS_publish(-1, "AA00")

	def ROS_publish(self, location, plate):
		location = str(location)
		plate = str(plate)

		message = str(self.teamID+','+self.password+','+location+','+plate)

		self.license_pub.publish(message)		

		# publish stall number
		self.stall_guess_pub.publish(str(location))
