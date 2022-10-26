import serial
from kin import kinematics
import math

class fuzzy_set:
	FUZZY_SET = []

	def __init__(self, set_name, set_xstart, set_xend, set_element_names):
		# odd even result of (val % 2)
		ODD = 1
		EVEN = 0

		self.set_name = set_name

		num_elements = len(set_element_names)
		half_num_elements = num_elements / 2
		
		# next_half_point example, if set_xend = 12, set_xstart = 6, and
		# num_elements = 7, then
		# (12-6) / (7-1) = 2
		next_half_point = (set_xend - set_xstart) / (num_elements - 1)

		# next point is the end of the set elements function when it goes
		# from 0 -> 1 -> 0
		# so keeping with previous comment about next_half_point, we see the
		# next point is 4.
		next_point = next_half_point * 2

		
		# get first half of membership function points, every other starting
		# at index 0
		# for example: {FL, L, CL, IF, CR, R, FR} sets the elements for
		#			   {FL, CL, CR, FR}
		k = set_xstart
		name = 0
		for i in range(0, math.ceil(half_num_elements)):
			fuzzy_name = set_element_names[name]

			xstart = k
			if (((num_elements % 2) == ODD and
				i == math.ceil(half_num_elements) - 1) or
				i == 0
			):
				xend = xstart + next_half_point
				self.FUZZY_SET.append({"name"  : fuzzy_name,
									   "xstart": xstart,
									   "xend"  : xend,
									   "xmid"  : xstart + (next_half_point / 2) 
									  })
			else:
				xend = xstart + next_point
				self.FUZZY_SET.append({"name"  : fuzzy_name,
									   "xstart": xstart,
									   "xend"  : xend,
									   "xmid"  : xstart + next_half_point
									  })

			k = xend
			name += 2

		# get second half of membership function points, every other starting
		# at index 1
		# for example: {FL, L, CL, IF, CR, R, FR} sets the elements for
		#			   {L, IF, R}
		k = set_xstart
		name = 1
		for i in range(0, math.floor(half_num_elements)):
			fuzzy_name = set_element_names[name]

			xstart = k
			if ((num_elements % 2) == EVEN and  
				i == math.floor(half_num_elements) - 1):
				xend = xstart + next_half_point
				self.FUZZY_SET.append({"name"  : fuzzy_name,
									   "xstart": xstart,
									   "xend"  : xend,
									   "xmid"  : xstart + (next_half_point / 2)
									  })
			else:
				xend = xstart + next_point
				self.FUZZY_SET.append({"name"  : fuzzy_name,
									   "xstart": xstart,
									   "xend"  : xend,
									   "xmid"  : xstart + next_half_point
									  })
			k = xend
			name += 2

		# sets FUZZY_SET in the same order as the set of element names passed
		# into init. The lamba function returns i as the key.
		#
		# The .index method tells you what the index of the value is into the 
		# method call. So it searches the list and finds the first index that 
		# matches that value and returns it.
		#
		# key is a callback function. When you call the sorted you pass a
		# function to it for it to call back into at some point. It is saying
		# when this happens call back into that function. The lambda function
		# is a callback function. Its because you dont always know what to do
		# and need to know without exiting the routine, in this case its
		# sorted.
		#
		# So every time sorted goes into a new index it calls the callback
		# function key. So everytime sorted iterated to a new item it calls
		# the function.
		#
		# See https://gist.github.com/sedihglow/770ed4e472935c5ab302d069b64280a8
		# for some code showing a implementation of sorted and execution. You
		# can change things to a list of lists, tuples, whatever and see
		# what it does and how it handles the callback function
		self.FUZZY_SET = sorted(
			self.FUZZY_SET,
			key=lambda i: set_element_names.index(i["name"])
		)

class fuzzy_controller:
	def __init__(self, arduino_serial, kin):
		self.arduino_serial = arduino_serial
		self.kin = kin

		# lists of the names for elements in the fuzzy set, used to set
		# names of the fuzzy set and also used in fuzzy_set to sort its
		# resulting list of dictionaries that stores the set element info
		self.fuzzy_hand_names = ["at (AT)", "not at (NAT)"]
		self.fuzzy_base_names = [
								 "far left (FL)", 
								 "left (L)", 
								 "close left (CL)",
								 "in front (IF)", 
								 "close right (CR)",
								 "right (R)",
								 "far right (FR)"
							   ]
		self.fuzzy_arm_names = [
							    "far behind hand (FBH)",
								"close behind hand (CBH)",
								"very close behind hand (VCBH)",
								"at hand (AH)",
								"very close front hand (VCFH)",
								"close front hand (CFH)",
								"far front hand (FFH)"
							   ]

		self.fuzzy_arm_set  = fuzzy_set("arm", 0, 10, self.fuzzy_arm_names)
		self.fuzzy_base_set = fuzzy_set("base", 0, 12, self.fuzzy_base_names)
		self.fuzzy_hand_set = fuzzy_set("hand", 0, 1, self.fuzzy_hand_names)
		

