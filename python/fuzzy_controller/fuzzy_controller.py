import serial
from kin import kinematics
import math
import clear as term

class fuzzy_set:
	def __init__(self, set_name, set_xstart, set_xend, set_element_names,
				 next_half_point, next_point):
		self.FUZZY_SET = []

		# odd even result of (val % 2)
		ODD = 1
		EVEN = 0

		self.SET_NAME = set_name

		self.NUM_ELEMENTS = len(set_element_names)
		half_num_elements = self.NUM_ELEMENTS / 2

		# get first half of membership function points, every other starting
		# at index 0
		# for example: {FL, L, CL, IF, CR, R, FR} sets the elements for
		#			   {FL, CL, CR, FR}
		k = set_xstart
		name = 0
		for i in range(0, math.ceil(half_num_elements)):
			fuzzy_name = set_element_names[name]

			xstart = k
			if (((self.NUM_ELEMENTS % 2) == ODD and
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
			if ((self.NUM_ELEMENTS % 2) == EVEN and
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

	def print_fuzzy_set(self):
		print(f"\n--- Fuzzy set {self.SET_NAME} ---")

		for i in range(0, self.NUM_ELEMENTS):
			for key, value in self.FUZZY_SET[i].items():
				print(f"{key}: {value}")
			print() # \n

class fuzzy_controller:
	def __init__(self, arduino_serial, kin):
		self.arduino_serial = arduino_serial
		self.kin = kin

		# lists of the names for elements in the fuzzy set, used to set
		# names of the fuzzy set and also used in fuzzy_set to sort its
		# resulting list of dictionaries that stores the set element info
		self.FUZZY_HAND_NAMES = ["at (AT)", "not at (NAT)"]

		# set output values for hand elements
		# if membership is 1 in AT, gripper fully closes.
		# if membership is 1 in NAT, gripper fully opens.
		# TODO: Honestly the end effector should be binary and not fuzzy,
		#		but for the sake of example we will keep it fuzzy for now
		#		and give NAT a low output set value. With how the fuzzy set
		#		is setup, the end effector may close before surrounding an
		#		object it may be trying to grab
		self.FUZZY_HAND_OUT_SET_VAL = (73, 1)

		# hand name indicies
		self.AT, self.NAT = [0, 1]

		self.FUZZY_BASE_NAMES = [
								 "far left (FL)",
								 "left (L)",
								 "close left (CL)",
								 "in front (IF)",
								 "close right (CR)",
								 "right (R)",
								 "far right (FR)"
							   ]
		# set output values for base elements
		self.FUZZY_BASE_OUT_SET_VAL = (20, 10, 1, 0, 1, 10, 20)

		# base names indicies
		(self.FL, self.L, self.CL, self.IF,
		 self.CR, self.R, self.FR) = [
								         0, 1, 2, 3, 4, 5, 6
									 ]

		self.FUZZY_ARM_NAMES = [
							    "far behind hand (FBH)",
								"close behind hand (CBH)",
								"very close behind hand (VCBH)",
								"at hand (AH)",
								"very close front hand (VCFH)",
								"close front hand (CFH)",
								"far front hand (FFH)"
							   ]
		# set output values for arm elements
		self.FUZZY_ARM_OUT_SET_VAL = (20, 10, 1, 0, 1, 10, 20)

		# arm names indicies
		(self.FBH, self.CBH, self.VCBH, self.AH,
		 self.VCFH, self.CFH, self.FFH) = [
										      0, 1, 2, 3, 4, 5, 6
										  ]

		# name, xstart and xend for each defined set
		self.ARM_NAME    = "arm"
		self.ARM_XSTART  = 0
		self.ARM_XEND    = 12
		self.BASE_NAME   = "base"
		self.BASE_XSTART = 0
		self.BASE_XEND   = 12
		self.HAND_NAME   = "hand"
		self.HAND_XSTART = 0
		self.HAND_XEND   = 1

		# next_half_point example, if set_xend = 12, set_xstart = 6, and
		# num_elements = 7, then
		# (12-6) / (7-1) = 2 where 2 is the half point of the triangle
		# membership function
		num_elements = len(self.FUZZY_ARM_NAMES)
		next_half_point = (
			(self.ARM_XEND - self.ARM_XSTART) / (num_elements - 1)
		)

		# next point is the end of the set elements function when it goes
		# from 0 -> 1 -> 0
		# so keeping with previous comment about next_half_point, we see the
		# next point is 4.
		next_point = next_half_point * 2

		# --- define fuzzy sets for braccio ---
		# setup whole arm (not hand and base) fuzzy set
		self.fuzzy_arm_set  = fuzzy_set(
										self.ARM_NAME,
										self.ARM_XSTART,
										self.ARM_XEND,
										self.FUZZY_ARM_NAMES,
										next_half_point,
										next_point
									   )

		# setup rotating base fuzzy set
		num_elements = len(self.FUZZY_BASE_NAMES)
		next_half_point = (
			(self.BASE_XEND - self.BASE_XSTART) / (num_elements - 1)
		)
		next_point = next_half_point * 2
		self.fuzzy_base_set = fuzzy_set(
										self.BASE_NAME,
										self.BASE_XSTART,
										self.BASE_XEND,
										self.FUZZY_BASE_NAMES,
										next_half_point,
										next_point
									   )

		# setup end effector fuzzy set
		num_elements = len(self.FUZZY_HAND_NAMES)
		next_half_point = (
			(self.HAND_XEND - self.HAND_XSTART) / (num_elements - 1)
		)
		next_point = next_half_point * 2
		self.fuzzy_hand_set = fuzzy_set(
										self.HAND_NAME,
										self.HAND_XSTART,
										self.HAND_XEND,
										self.FUZZY_HAND_NAMES,
										next_half_point,
										next_point
    								   )

	def get_hand_membership(self, x_in):
		x = x_in
		if (self.HAND_XSTART > x_in):
			x = self.HAND_XSTART
		if (self.HAND_XEND < x_in):
			x = self.HAND_XEND

		return (
					{
						"name": self.fuzzy_hand_set.FUZZY_SET[0]["name"],
						"membership": 1 - x
					},
					{
						"name": self.fuzzy_hand_set.FUZZY_SET[1]["name"],
						"membership": x
					}
			   )

	def get_membership(self, fuzzy_set, x_in):
		# set a local x_in to the limits of the set so there are less cases to
		# evaluate and can just use the "middle" equation of the set to get
		# 1, 0, or inbetween as the membership function without altering the
		# original x_in for the caller
		x = x_in
		if (fuzzy_set is self.fuzzy_hand_set):
			return self.get_hand_membership(x_in)
		elif (fuzzy_set is self.fuzzy_arm_set):
			if (self.ARM_XSTART > x_in):
				x = self.ARM_XSTART
			elif (self.ARM_XEND < x_in):
				x = self.ARM_XEND
		elif (fuzzy_set is self.fuzzy_base_set):
			if (self.BASE_XSTART > x_in):
				x = self.BASE_XSTART
			elif (self.BASE_XEND < x_in):
				x = self.BASE_XEND
		else:
			print("Error: Invalid fuzzy set sent to get_membership()")
			return None


		found = False
		i = 0
		while (not found and  i < fuzzy_set.NUM_ELEMENTS):
			i_xstart = fuzzy_set.FUZZY_SET[i]["xstart"]
			i_xend   = fuzzy_set.FUZZY_SET[i]["xend"]

			# find what sets x_in belongs to

			# i == 0 and i == NUM_ELEMENTS-1 are special cases with how the
			# fuzzy set is built.

			# when i == 0 and x_in is in the set element, the other set element
			# in the membership for x is i == 1 (or the next element)
			if (i == 0 and i_xstart <= x and i_xend >= x):
				found = True
				membership = (
								{
									"name": fuzzy_set.FUZZY_SET[i]["name"],
									"membership": None,
									"x_position": "special right"
								},
								{
									"name": fuzzy_set.FUZZY_SET[i+1]["name"],
									"membership": None,
									"x_position": "left"
								}
							 )
			# when i == NUM_ELEMENTS-1 and x_in is in the set element, the
			# other set element in the membership for x is i == i-1
			elif (i == fuzzy_set.NUM_ELEMENTS - 1 and
				  i_xstart <= x and
				  i_xend >= x
			):
				found = True
				membership = (
								{
									"name": fuzzy_set.FUZZY_SET[i]["name"],
									"membership": None,
									"x_position": "special left"
								},
								{
									"name": fuzzy_set.FUZZY_SET[i-1]["name"],
									"membership": None,
									"x_position": "right"
								}
							 )
			elif (i_xstart <= x and i_xend >= x):
				# find second set x_in belongs to
				i_xmid = fuzzy_set.FUZZY_SET[i]["xmid"]
				if (i_xstart <= x and i_xmid >= x): # prev set element
					found = True
					membership = (
									{
										"name": fuzzy_set.FUZZY_SET[i]["name"],
										"membership": None,
										"x_position": "left"
									},
									{
										"name": fuzzy_set.FUZZY_SET[i-1]["name"],
										"membership": None,
										"x_position": "right"
									}
								)

				else: # (i_xend >= x and i_xmid <= x) # next set element
					found = True
					membership = (
									{
										"name": fuzzy_set.FUZZY_SET[i]["name"],
										"membership": None,
										"x_position": "right"
									},
									{
										"name": fuzzy_set.FUZZY_SET[i+1]["name"],
										"membership": None,
										"x_position": "left"
									}
								)
			i += 1
		# end while

		if (not found):
			print("Error: Could not find x_in in range of xstart/xend\n")
			return None


		if ((membership[0]["name"] in self.FUZZY_ARM_NAMES or
			membership[0]["name"] in self.FUZZY_BASE_NAMES) and
			(membership[1]["name"] in self.FUZZY_ARM_NAMES or
			membership[1]["name"] in self.FUZZY_BASE_NAMES)
		):
			self.fill_arm_base_membership(membership, x)
		else:
			print("membership name not in fuzzy base or arm names. Invalid set\n"
				  "name in membership.")

		return membership

	# set membership value in membership dict
	#
	# fuzzy_base and fuzzy_arm sets have the same membership functions
	# in current setup
	#
	# - if the x_position is on the left, then the other set element for
	#	x_in will always be the previous one
	#
	# - if the x_position of the first membership is on the right then the
	#   other set element for x_in will always be the next one, which also
	#   means for the second membership equation it will always be the left
	#   half of the next set element
	#
	#   x should be set to the xstart or xend or inbetween based on the fuzzy
	#   set. This function is called in get_membership() and if we call it
	#   from anywhere else the same x_in checking should be done before
	#   passing to this function.
	#
	#   TODO: Decide on checking x's range in this function as well, though
	#		  with current call of this function in get_membership() its
	#		  redundant
	def fill_arm_base_membership(self, membership, x):
		if (membership[0]["name"] == self.FUZZY_ARM_NAMES[self.FBH] or
			membership[0]["name"] == self.FUZZY_BASE_NAMES[self.FL]):
			# FBH/FL
			membership[0]["membership"] = (2 - x) / 2
			# CBH/L left
			membership[1]["membership"] =  x / 2
		elif (membership[0]["name"] == self.FUZZY_ARM_NAMES[self.CBH] or
			  membership[0]["name"] == self.FUZZY_BASE_NAMES[self.L]):
			if (membership[0]["x_position"] == "left"):
				# CBH/L left
				membership[0]["membership"] = x / 2
				# FL/FBH (special right)
				membership[1]["membership"] = (2 - x) / 2

			else: # x_position == right
				# CBH/L right
				membership[0]["membership"] = (4 - x) / 2
				# VCBH/CL left
				membership[1]["membership"] = (x - 2) / 2

		elif (membership[0]["name"] == self.FUZZY_ARM_NAMES[self.VCBH] or
		      membership[0]["name"] == self.FUZZY_BASE_NAMES[self.CL]):
			if (membership[0]["x_position"] == "left"):
				# VCBH/CL left
				membership[0]["membership"] = (x - 2) / 2
				# CBH/L right
				membership[1]["membership"] = (4 - x) / 2

			else: # x_position == right
				# VCBH/CL right
				membership[0]["membership"] = (6 - x) / 2
				# AH/IF left
				membership[1]["membership"] = (x - 4) / 2

		elif (membership[0]["name"] == self.FUZZY_ARM_NAMES[self.AH] or
			  membership[0]["name"] == self.FUZZY_BASE_NAMES[self.IF]):
			if (membership[0]["x_position"] == "left"):
				# AH/IF left
				membership[0]["membership"] = (x - 4) / 2
				# VCBH/CL right
				membership[1]["membership"] = (6 - x) / 2

			else: # x_position == right
				# AH/IF right
				membership[0]["membership"] = (8 - x) / 2
				# VCFH/CR left
				membership[1]["membership"] = (x - 6) / 2

		elif (membership[0]["name"] == self.FUZZY_ARM_NAMES[self.VCFH] or
			  membership[0]["name"] == self.FUZZY_BASE_NAMES[self.CR]):
			if (membership[0]["x_position"] == "left"):
				# VCFH/CR left
				membership[0]["membership"] = (x - 6) / 2
				# AH/IF right
				membership[1]["membership"] = (8 - x) / 2

			else: # x_position == right
				# VCFH/CR right
				membership[0]["membership"] = (10 - x) / 2
				# CFH/R left
				membership[1]["membership"] = (x - 8) / 2

		elif (membership[0]["name"] == self.FUZZY_ARM_NAMES[self.CFH] or
			  membership[0]["name"] == self.FUZZY_BASE_NAMES[self.R]):
			if (membership[0]["x_position"] == "left"):
				# CFH/R left
				membership[1]["membership"] = (x - 8) / 2
				# VCFH/CR right
				membership[1]["membership"] = (10 - x) / 2

			else: # x_position == right
				# CFH/R right
				membership[1]["membership"] = (12 - x) / 2
				# FFH/FR (special left)
				membership[1]["membership"] = (x - 10) / 2

		elif (membership[0]["name"] == self.FUZZY_ARM_NAMES[self.FFH] or
			  membership[0]["name"] == self.FUZZY_BASE_NAMES[self.FR]):
			# FFH/FR
			membership[0]["membership"] = (x - 10)
			# CFH/R right
			membership[1]["membership"] = (12 - x) / 2

		else:
			print("membership val not filled, invalid name in passed member\n")

		return membership

	def print_membership(self, membership):
		print("-- membership values --")
		i = 0
		for member in membership:
			print(f"-- set {i} --")
			for key, value in member.items():
				print(f"{key}: {value}")
			print();
			i += 1

	def membership_test(self):
		HAND_MENU_IN = 1
		ARM_MENU_IN  = 2
		BASE_MENU_IN = 3
		ALL_MENU_IN  = 4
		EXIT_VAL     = 5
		MEMBER_MENU_MIN = 1
		MEMBER_MENU_MAX = 5

		stay_flag = True
		while (stay_flag):
			in_range = False
			digit = False
			while (not digit or not in_range):
				term.clear()
				print("\n--- Testing membership functionality ---\n"
					  "This section is to test the membership calculation\n"
					  "functionality for the fuzzy sets. Enter an x value to\n"
					  "be evaluated for membership in a given fuzzy set.\n")

				print("1. Hand set - eval for the end effector fuzzy set\n"
					  "2. Arm set  - eval for the whole arm fuzzy set\n"
					  "3. Base set - eval for the rotating base fuzzy set\n"
					  "4. All sets - eval for all fuzzy sets\n"
					  "5. exit")
				read = input("Enter number: ")

				digit = read.isdigit()
				if (digit):
					menu_input = int(read)
					if (menu_input >= MEMBER_MENU_MIN and
						menu_input <= MEMBER_MENU_MAX):
						in_range = True
					else:
						print("Invalid input\n")
						input("-- Press Enter to Continue --")
				else:
					print("Invalid input\n")
					input("-- Press Enter to Continue --")
		    # end while

			if (menu_input == EXIT_VAL):
				stay_flag = False
			else: # continue with execution
				xval = self.get_user_crisp_input()

				# get membership and print results
				if (menu_input == HAND_MENU_IN):
					hand_membership = self.get_membership(self.fuzzy_hand_set,
														  xval)
					if (hand_membership):
						term.clear()
						print("-- fuzzy hand set membership --")
						self.print_membership(hand_membership)
				elif (menu_input == ARM_MENU_IN):
					arm_membership = self.get_membership(self.fuzzy_arm_set,
														 xval)
					if (arm_membership):
						term.clear()
						print("-- fuzzy arm set membership --")
						self.print_membership(arm_membership)
				elif (menu_input == BASE_MENU_IN):
					base_membership = self.get_membership(self.fuzzy_base_set,
														  xval)
					if (base_membership):
						term.clear()
						print("-- fuzzy base set membership --")
						self.print_membership(base_membership)
				else: # menu_input == ALL_MENU_IN
					hand_membership = self.get_membership(self.fuzzy_hand_set,
														  xval)
					arm_membership = self.get_membership(self.fuzzy_arm_set,
														  xval)
					base_membership = self.get_membership(self.fuzzy_base_set,
														  xval)

					# if successfully got membership, print result
					if (hand_membership):
						term.clear()
						print("-- fuzzy hand set membership --")
						self.print_membership(hand_membership)
						input("-- Press Enter to Continue --")
					else:
						print("Error getting hand membership\n")

					if (arm_membership):
						term.clear()
						print("-- fuzzy arm set membership --")
						self.print_membership(arm_membership)
						input("-- Press Enter to Continue --")
					else:
						print("Error getting arm membership\n")

					if (base_membership):
						term.clear()
						print("-- fuzzy base set membership --")
						self.print_membership(base_membership)
					else:
						print("Error getting base membership\n")

				input("-- Press Enter to Continue --")
		# end while

	def print_fuzzy_sets(self):
		PRINT_HAND_IN = 1
		PRINT_ARM_IN  = 2
		PRINT_BASE_IN = 3
		PRINT_ALL_IN  = 4
		EXIT_VAL_IN   = 5
		MENU_MIN_IN   = 1
		MENU_MAX_IN   = 5

		stay_flag = True
		while (stay_flag):
			in_range = False
			digit = False
			while (not digit or not in_range):
				term.clear()
				print("\n---- Print fuzzy set operation ---\n"
					  "1. Print Hand fuzzy set\n"
					  "2. Print Arm fuzzy set\n"
					  "3. Print Base fuzzy set\n"
					  "4. Print All fuzzy sets\n"
					  "5. exit")
				reading = input("Enter number: ")

				digit = reading.isdigit()
				if (digit):
					reading = int(reading)
					if (reading >= MENU_MIN_IN and reading <= MENU_MAX_IN):
						in_range = True
					else:
						print("Invalid input\n")
						input("-- Press Enter to Continue --")
				else:
					print("Invalid input\n")
					input("-- Press Enter to Continue --")

			if (reading == PRINT_HAND_IN):
				term.clear()
				self.fuzzy_hand_set.print_fuzzy_set()
			elif (reading == PRINT_ARM_IN):
				term.clear()
				self.fuzzy_arm_set.print_fuzzy_set()
			elif (reading == PRINT_BASE_IN):
				term.clear()
				self.fuzzy_base_set.print_fuzzy_set()
			elif (reading == PRINT_ALL_IN):
				term.clear()
				self.fuzzy_hand_set.print_fuzzy_set()
				input("-- Press Enter for Next Set --")

				term.clear()
				self.fuzzy_arm_set.print_fuzzy_set()
				input("-- Press Enter for Next Set --")

				term.clear()
				self.fuzzy_base_set.print_fuzzy_set()

			if (reading == EXIT_VAL_IN):
				stay_flag = False
			else:
				input("-- Press Enter to Continue --")

	# gets a crisp input from the user instead of a sensor.
	def get_user_crisp_input(self):
		afloat = False
		while (not afloat):
			print("\nNOTE: Hand set range: 0-1\n"
					"      Arm  set range: 0-12\n"
					"      Base set range: 0-12\n"
					"      x can be out of this range\n")
			read = input("Enter x input value (as if from sensor): ")

			try:
				xval = float(read)
				afloat = True
			except ValueError:
				print("Invalid input")
				input("-- Press Enter to Continue --")

		return xval

	def get_crisp_outputs(self, fuzzy_set):
		if (fuzzy_set is self.fuzzy_hand_set):
			# get crisp output for hand set
			return None
		elif (fuzzy_set is self.fuzzy_arm_set):
			# get crisp output for arm set
			return None
		elif (fuzzy_set is self.fuzzy_base_set):
			# get crisp output for base set
			return None
		else:
			print("Error: Invalid fuzzy set sent to get_crisp_outputs()")
			return None

		return None

	def fuzzy_controller_exec(self):
		return None
