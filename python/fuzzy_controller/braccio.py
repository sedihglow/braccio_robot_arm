from command import command_interface
from arduino_serial import arduino_com
from kin import kinematics
from fuzzy_controller import fuzzy_controller

class braccio_interface:
    EXIT_FLAG_RET = False # Exit value to exit from menu or program
    STAY_FLAG_RET = True # return value for interfaces staying in program
    NUM_SERVOS = 6

    def __init__(self, verbose, port, baudrate, rtimeout):
        self.verbose = verbose
		
        self.arduino_serial = arduino_com(port, baudrate, rtimeout)

        self.cmd = command_interface(verbose)
        self.kin = kinematics()

        self.fuzzy_con = fuzzy_controller(self.arduino_serial,
                                          self.kin)
    
    def verbose_print(self, msg):
        if (self.verbose):
            print(msg)
    
    # Starts communication with the braccio controller and gets init angles
    def begin_com(self):
        self.arduino_serial.begin()

        # get setup messages to confirm Arduino is on
        self.read_exec()

        # init angles from Arduino
        msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
        self.arduino_serial.write(msg)
        self.read_exec()

    # Read a message from the braccio controller.
    # NOTE: Loops waiting for the finish sending command from the controller. 
    #       Only call when something should be returning from the controller.
	# TODO: Make a timer so if there is no response it moves on or calls
	#		an error.
    def read_exec(self):
        finished = False
        while (not finished):
            read = self.arduino_serial.read()
            if (read):
                msg_size = int.from_bytes(read, byteorder="little", 
                                          signed=False)
                read = self.arduino_serial.read(msg_size)
                p_msg = self.cmd.parse_in_msg(read)
                if (p_msg[0] == self.cmd.ACK):
                    self.verbose_print("ACK recieved")
                elif (p_msg[0] == self.cmd.PRINT_MSG):
                    self.cmd.exec_print(p_msg)
                elif(p_msg[0] == self.cmd.CMD_MSG):
                    self.cmd.exec_command(p_msg, self.kin.angles)
                    self.kin.set_kin_vars()
                elif(p_msg[0] == self.cmd.FINISH):
                    self.verbose_print("Arduino finished sending message")
                    finished = True

    # Directs the user to various interfaces and options for the braccio robot
    # arm. 
    def interface_director(self):
        EXIT_VAL = 4
        CMD_INTER_VAL  = 1 # cmd interface val
        KIN_INTER_VAL  = 2 # kinematics interface val
        FUZZY_CONT_VAL = 3 # fuzzy controller val

        print("\nThis program will allow you to tinker with all the servos\n"
              "via the command interface with the Arduino controller.\n"
              "It will also showcase an implementation of kinimatics\n"
              "for this Braccio robot arm")

        digit = False
        while (not digit):
            # print menu for cmd or inverse kin
            print("\nWhich functionality would you like to run?") 
            print("1. Command Interface (Tinker with servos and commands)\n"
                  "2. Kinematics\n"
                  "3. Fuzzy Controller (Example of fuzzy controller w/ braccio)\n"
                  "4. exit")
            read = input("Enter Number: ")

            digit = read.isdigit()
            if (digit):
                read = int(read)

        if (read == EXIT_VAL):
            return self.EXIT_FLAG_RET;

        if (read == CMD_INTER_VAL): # cmd interface
            stay_flag = self.STAY_FLAG_RET
            while (stay_flag):
                stay_flag = self.cmd_menu_input_send()

                print("\nreading/exec messages from Arduino")
                if (stay_flag):
                    self.read_exec()
        elif (read == KIN_INTER_VAL): # inverse kin
            stay_flag = self.STAY_FLAG_RET
            while (stay_flag):
                stay_flag = self.kin_interface()
        elif (read == FUZZY_CONT_VAL): # fuzzy controller example
            stay_flag = self.STAY_FLAG_RET
            while (stay_flag):
                stay_flag = self.fuzzy_controller_interface()
        else:
            print("Invalid input\n")
            
        return self.STAY_FLAG_RET;
   
    # fills kin.angles with user input
    def get_user_angles(self):
        digit = False
        while (not digit):
            print("\nEnter 6 angles for braccio separated with commas," 
                  "servo 0-5 (M1-M6)")
            angles = input("Enter angles (M1, M2, M3, M4, M5, M6)"
                           ": ").split(", ")

            i = 0 
            while (i < self.NUM_SERVOS and angles[i].isdigit()):
                angles[i] = int(angles[i])
                i += 1
            
            # if all angles were digits and converted, break loop
            if (i == self.NUM_SERVOS):
                digit = True
            else:
                print(f"Invalid input - {angles[i]}\n")
        
        for i in range(0, self.NUM_SERVOS):
            self.kin.angles[i] = angles[i]
    
    # Get user angles for kin.angles or use the current angles from braccio
    def input_current_or_new_angles(self):
        CURRENT_ANGLES = 1
        USER_ANGLE_VAL = 2
        MENU_IN_MIN = 1
        MENU_IN_MAX = 2
        
        in_range = False
        digit = False
        while (not digit and not in_range): 
            print("\nUse current Braccio angles or use user input angles?")
            print("1. Current angles.\n"
                  "2. User input angles.")
            read = input("Enter number: ")
            
            digit = read.isdigit()
            if (digit):
                read = int(read)
                if (read >= MENU_IN_MIN and read <= MENU_IN_MAX):
                    in_range = True
                else:
                    print("Invalid input\n")
            else:
                print("Invalid input\n")

        if (read == USER_ANGLE_VAL):
            self.get_user_angles()
        else: #(read == CURRENT_ANGLES):
            print("Using Braccio's current angles")
    
    def print_cmd_menu(self):
        print("\nChoose angle to set or command to send\n"
              "1. m1, base\n"
              "2. m2, shoulder\n"
              "3. m3, elbow\n"
              "4. m4, wrist vertical\n"
              "5. m5, wrist rotation\n"
              "6. m6, gripper\n"
              "7. All angles\n"
              "8. Request all angles\n"
              "9. Set default position\n"
              "10. exit")

    # Interface for controller commands to change angles on the braccio. Gets
    # user input, builds the corresponding message and writes to the controller.
    # Once written, request the angles from the controller to make sure angles
    # being recorded on host and controller are consistant.
    def cmd_menu_input_send(self):
        # Menu option values based on print_cmd_menu() options
        MIN_OPTS     = 1
        MAX_OPTS     = 10
        M1_BASE      = 1
        M2_SHOULDER  = 2
        M3_ELBOW     = 3
        M4_WRIST_V   = 4
        M5_WRIST_R   = 5
        M6_GRIPPER   = 6
        ALL_ANGLES   = 7
        REQUEST_ANGS = 8
        SET_DFLT_POS  = 9
        EXIT_PROGRAM = 10

        digit = False
        while (not digit):
            self.print_cmd_menu()
            change_angle = input("Enter number: ")

            digit = change_angle.isdigit()
            if (digit):
                change_angle = int(change_angle)

        if (change_angle > MAX_OPTS or change_angle < MIN_OPTS):
            print("\nInvalid Input")
            return self.STAY_FLAG_RET

        if (change_angle == EXIT_PROGRAM):
                return self.EXIT_FLAG_RET
        
        if (change_angle == ALL_ANGLES):
            digit = False
            while (not digit):
                angles = input("Enter angles (M1, M2, M3, M4, M5, M6)"
                               ": ").split(', ')

                i = 0 
                while (i < self.NUM_SERVOS and angles[i].isdigit()):
                    angles[i] = int(angles[i])
                    i = i + 1
                    
                # if all angles were digits and converted, break loop
                if (i == self.NUM_SERVOS):
                    digit = True
                else:
                    print(f"Invalid input - {angles[i]}\n")

            msg = self.cmd.build_cmd_msg(self.cmd.MX_ANGLE, angles[0], 
                                         angles[1], angles[2], angles[3],
                                         angles[4], angles[5])

        elif (change_angle == REQUEST_ANGS):
            msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
            self.arduino_serial.write(msg)
            return self.STAY_FLAG_RET
        elif (change_angle == SET_DFLT_POS):
            msg = self.cmd.build_cmd_msg(self.cmd.SET_DFLT_POS)
        else:
            digit = False
            while (not digit):
                angle = input("Enter angle: ")

                digit = angle.isdigit()
                if (digit):
                    angle = int(angle)
                else:
                    print("Invalid input\n")

            if (change_angle == M1_BASE):
                msg = self.cmd.build_cmd_msg(self.cmd.M1_ANGLE, angle)
            elif (change_angle == M2_SHOULDER):
                msg = self.cmd.build_cmd_msg(self.cmd.M2_ANGLE, angle)
            elif (change_angle == M3_ELBOW):
                msg = self.cmd.build_cmd_msg(self.cmd.M3_ANGLE, angle)
            elif (change_angle == M4_WRIST_V):
                msg = self.cmd.build_cmd_msg(self.cmd.M4_ANGLE, angle)
            elif (change_angle == M5_WRIST_R):
                msg = self.cmd.build_cmd_msg(self.cmd.M5_ANGLE, angle)
            elif (change_angle == M6_GRIPPER):
                msg = self.cmd.build_cmd_msg(self.cmd.M6_ANGLE, angle)

        self.arduino_serial.write(msg)

        # retrieve changed angles from arduino to ensure it matches in the class
        msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
        self.arduino_serial.write(msg)
        return self.STAY_FLAG_RET

    def kin_menu(self):
        print("\nKinematics functionalities")
        print("1. Rotation Matrix function\n"
              "2. Displacement Vectors\n"
              "3. Homogeneous Transform Matrix.\n"
              "4. exit")
 
    # Interface to execute kinematics, forward and inverse.
    def kin_interface(self):
        # menu inputs based on kin_menu() 
        ROT_MAT_IN    = 1 # Rotation Matrix Functionality input
        DISP_VECT_IN  = 2 # Displacement Vectors input
        HOMO_TRANS_IN = 3 # Homogeneous Transform Matrix Funcionality input
        EXIT_VAL_IN   = 4
        KIN_MENU_MIN  = 1
        KIN_MENU_MAX  = 4
        END_ROT_IN_MIN = 1
        END_ROT_IN_MAX = 5
        START_ROT_IN_MIN = 0
        START_ROT_IN_MAX = 4
        
        in_range = False
        digit = False
        while (not digit and not in_range):
            self.kin_menu()
            read = input("Enter number: ")
            
            digit = read.isdigit() 
            if (digit):
                read = int(read)
                if (read >= KIN_MENU_MIN or read <= KIN_MENU_MAX):
                    in_range = True
                else:
                    print("Invalid input\n")
            else:
                print("Invalid input\n")
        if (read == EXIT_VAL_IN):
            return self.EXIT_FLAG_RET
        
        if (read == ROT_MAT_IN): # rotation matrix functionality testing
            print("--- Testing rotation matrix function ---")
            
            self.input_current_or_new_angles()
            
            in_range = False
            digit = False
            while (not digit and not in_range):
                print("\nEnter starting frame for rot matrix")
                start_frame = input("Enter starting frame number (0-4): ")

                digit = start_frame.isdigit()
                if (digit):
                    start_frame = int(start_frame)
                    if (start_frame >= START_ROT_IN_MIN and 
                        start_frame <= START_ROT_IN_MAX):
                        in_range = True
                    else
                        print("Invalid input\n")
                else:
                    print("Invalid input\n")
            
            in_range = False
            digit = False
            while (not digit and not in_range):
                print("\nEnter ending frame for rot matrix")
                end_frame = input("Enter ending frame number (1-5): ")

                digit = end_frame.isdigit()
                if (digit):
                    end_frame = int(end_frame)
                    if (end_frame >= END_ROT_IN_MIN and
                        end_frame <= END_ROT_IN_MAX):
                        in_range = True
                    else
                        print("Invalid input\n")

            rot_matrix = self.kin.create_rot_matrix(start_frame,end_frame)
            print("\n--rotation matrix {:d}_{:d}--".format(start_frame, 
                                                           end_frame))
            print(rot_matrix)
            
            # reset angles to match braccio
            msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
            self.arduino_serial.write(msg)
            self.read_exec()
        elif (read == DISP_VECT_IN): # test the displacement vector function
            print("\nTesting displacement vectors")
            
            self.input_current_or_new_angles()
            
            # set new displacement vectors and print
            self.kin.create_fill_disp_vects()
            self.kin.print_disp_vects()

            # reset angles and displacement vectors
            msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
            self.arduino_serial.write(msg)
            self.read_exec()
        elif (read == HOMO_TRANS_IN): # Homogeneous transform functionality
            print("\nTesting the Homogeneous Transform Matrix functionality.")
            self.input_current_or_new_angles()
            self.kin.set_kin_vars() # in case angles changed
            self.kin.print_homo_trans_mats()

        return self.STAY_FLAG_RET
   
    # Interface to demonstrate a fuzzy controller, currently there is no
    # sensor so user input is used instead.
    def fuzzy_controller_interface(self):
        FUZZY_CONT_EX = 1 # Fuzzy controller example
        PRINT_FUZZY_SETS = 2
        MEMBERSHIP_CALC_TEST = 3
        EXIT_VAL = 4
        FC_MENU_MIN = 1
        FC_MENU_MAX = 4
        
        in_range = False
        digit = False
        while (not digit and not in_range):
            print("\nThis section shows the example of a fuzzy logic\n"
                  "controller and print/testing functionalities\n")
                  
            print("1. Fuzzy logic example\n"
                  "2. Print Fuzzy Sets for Braccio\n"
                  "3. Membership calculator testing\n"
                  "4. exit")
            read = input("Enter number: ")
            
            digit = read.isdigit()
            if (digit):
                read = int(read)
                if (read >= FC_MENU_MIN and read <= FC_MENU_MAX):
                    in_range = True
                else:
                    print("Invalid input\n")
            else:
                print("Invalid input\n")

        if (read == EXIT_VAL):
            return self.EXIT_FLAG_RET

        if (read == FUZZY_CONT_EX):
            self.fuzzy_con.controller_exec()
            return self.STAY_FLAG_RET
        elif (read == PRINT_FUZZY_SETS):
            return self.STAY_FLAG_RET
        elif (read == MEMBERSHIP_CALC_TEST):
            self.fuzzy_con.membership_test()
            return self.STAY_FLAG_RET
        else:
            print("Invalid input\n")

        return self.STAY_FLAG_RET
