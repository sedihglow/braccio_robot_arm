import serial
from kin import kinematics

class fuzzy_controller:
	def __init__(self, arduino_serial, kin):
		print("\n\n------ IN FUZZY CONTROllER -----\n\n")
		self.arduino_serial = arduino_serial
		self.kin = kin
		print("\n\n------ exiting FUZZY CONTROllER -----\n\n")
