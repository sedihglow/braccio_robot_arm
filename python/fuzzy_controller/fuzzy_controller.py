import serial
from kin import kinematics
import math

class fuzzy_set:
	# -- Name/set of types of elements --
	# base definitions
	# names
	FL, L, CL, IF, CR, R, FR = ("front_left", "left", "close left", "in front",
								"close right", "right", "far right")
	# index numbers
	FL_INDEX, L_INDEX, CL_INDEX, IF_INDEX, CR_INDEX, R_INDEX, FR_INDEX = (
		0, 1, 2, 3, 4, 5, 6
	)
	FUZZ_SET = []

	def __init__(self, set_name, set_xstart, set_xend, set_element_names):
		self.set_name = set_name
		
		next_half_point = (set_xend - set_xstart) / (num_elements - 1)
		next_point = next_half_point * 2

		num_elements = len(set_element_names)
		
		# get first half of membership function points, every other starting
		# at index 0
		# for example: {FL, L, CL, IF, CR, R, FR} sets the elements for
		#			   {FL, CL, CR, FR}
		k = set_xstart
		name = 0
		for i in range(0, math.ceil(num_elements / 2)):
			xstart = k
			xend   = k + next_point
			fuzzy_name = set_element_names[name]
			self.FUZZ_SET[i].append({"name"  : fuzzy_name,
									 "xstart": xstart,
									 "xend"  : xend,
									 "midpoint": xend / xstart
									})
			k += next_point
			name += 2

		# get second half of membership function points, every other starting
		# at index 1
		# for example: {FL, L, CL, IF, CR, R, FR} sets the elements for
		#			   {L, IF, R}
		k = set_xstart + 1
		name = 1
		for i in range(0, math.floor(num_elements / 2)):
			xstart = k
			xend   = k + next_point
			fuzzy_name = set_element_names[name]
			self.FUZZ_SET[i].append({"name"  : fuzzy_name,
									 "xstart": xstart,
									 "xend"  : xend,
									 "midpoint": xend / xstart
									})
			k += next_point
			name += 2

		print(self.FUZZ_SET)


class fuzzy_controller:
	def __init__(self, arduino_serial, kin):
		self.arduino_serial = arduino_serial
		self.kin = kin

