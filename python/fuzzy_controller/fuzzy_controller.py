import serial
from kin import kinematics

class fuzzy_controller:
	def __init__(self, arduino_serial, kin):
		self.arduino_serial = arduino_serial
		self.kin = kin
