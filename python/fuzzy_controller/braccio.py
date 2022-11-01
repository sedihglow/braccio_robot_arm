from command import command_interface
from arduino_serial import arduino_com
from kin import kinematics
from fuzzy_controller import fuzzy_controller
import clear as term

class braccio_interface:
    EXIT_FLAG_RET = False # Exit value to exit from menu or program
    STAY_FLAG_RET = True # return value for interfaces staying in program
    NUM_SERVOS = 6

    def __init__(self, verbose, port, baudrate, rtimeout):
        self.verbose = verbose
        
        self.arduino_serial = arduino_com(port, baudrate, rtimeout)

        self.kin = kinematics()

        self.cmd = command_interface(verbose, self.arduino_serial, self.kin)

        self.fuzzy_con = fuzzy_controller(self.arduino_serial, self.kin,
                                          self.cmd)
    
    def print_verbose(self, msg):
        if (self.verbose):
            print(msg)
    
    # Starts communication with the braccio controller and gets init angles
    def begin_com(self):
        term.clear()
        self.arduino_serial.begin()

        # get setup messages to confirm Arduino is on
        self.cmd.read_exec()

        # init angles from Arduino
        msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
        self.arduino_serial.write(msg)
        self.cmd.read_exec()

    # Directs the user to various interfaces and options for the braccio robot
    # arm. 
    def interface_director(self):
        CMD_INTER_VAL  = 1 # cmd interface val
        KIN_INTER_VAL  = 2 # kinematics interface val
        FUZZY_CONT_VAL = 3 # fuzzy controller val
        EXIT_VAL = 4
        MENU_MIN_IN = 1
        MENU_MAX_IN = 4
        
        in_range = False
        digit = False
        while (not digit or not in_range):
            term.clear()
            print("\nThis program will allow you to control the braccio robot\n"
                  "arm and demonstrate various robotics topics with the arm\n"
                  "such as the following -\n"
                  "- Tinker with the servos of the braccio robot arm\n"
                  "- Demonstrate kinematics\n"
                  "- Demonstrate a fuzzy controller\n")

            print("\nWhich functionality would you like to run?\n"
                  "1. Command Interface (Tinker with servos and commands)\n"
                  "2. Kinematics\n"
                  "3. Fuzzy Controller (example with braccio)\n"
                  "4. exit")
            read = input("Enter Number: ")

            digit = read.isdigit()
            if (digit):
                read = int(read)
                if (read >= MENU_MIN_IN and read <= MENU_MAX_IN):
                    in_range = True
                else:
                    print("Invalid input\n")
                    input("-- Press Enter to Continue --")
            else:
                print("Invalid input\n")
                input("-- Press Enter to Continue --")

        if (read == EXIT_VAL):
            return self.EXIT_FLAG_RET;

        if (read == CMD_INTER_VAL): # cmd interface
            stay_flag = self.STAY_FLAG_RET
            while (stay_flag):
                stay_flag = self.cmd_interface()
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
            input("-- Press Enter to Continue --")
            
        return self.STAY_FLAG_RET;
   
    # fills kin.angles with user input
    def get_user_angles(self):
        digit = False
        while (not digit):
            angle_range = False
            while (not angle_range):
                print("\nEnter 6 angles for braccio separated with commas," 
                      "servo 0-5 (M1-M6)")
                angles = input("Enter angles (M1, M2, M3, M4, M5, M6)"
                               ": ").split(", ")
                if (len(angles) >= self.NUM_SERVOS):
                    angle_range = True
                else:
                    print("Invalid input\n")
                    input("-- Press Enter to Continue --")
            
            i = 0 
            while (i < self.NUM_SERVOS and angles[i].isdigit()):
                angles[i] = int(angles[i])
                i += 1
            
            # if all angles were digits and converted, break loop
            if (i == self.NUM_SERVOS):
                digit = True
            else:
                print(f"Invalid input - {angles[i]}\n")
                input("-- Press Enter to Continue --")
        
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
        first_pass = True
        while (not digit or not in_range):
            if (not first_pass):
                term.clear()
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
                    input("-- Press Enter to Continue --")
            else:
                print("Invalid input\n")
                input("-- Press Enter to Continue --")
            first_pass = False
        if (read == USER_ANGLE_VAL):
            self.get_user_angles()
        else: #(read == CURRENT_ANGLES):
            print("Using Braccio's current angles")
            input("-- Press Enter to Continue --")
    
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
    def cmd_interface(self):
        # Menu option values based on print_cmd_menu() options
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
        MENU_MIN_OPTS = 1
        MENU_MAX_OPTS = 10
        
        in_range = False
        digit = False
        while (not digit or not in_range):
            term.clear()
            self.print_cmd_menu()
            cmd_in = input("Enter number: ")

            digit = cmd_in.isdigit()
            if (digit):
                cmd_in = int(cmd_in)
                if (cmd_in >= MENU_MIN_OPTS and cmd_in <= MENU_MAX_OPTS):
                    in_range = True
                else:
                    print("Invalid input\n")
                    input("-- Press Enter to Continue --")
            else:
                print("Invalid input\n")
                input("-- Press Enter to Continue --")

        if (cmd_in == EXIT_PROGRAM):
            return self.EXIT_FLAG_RET
        elif (cmd_in == REQUEST_ANGS):
            msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
            self.arduino_serial.write(msg)
            self.cmd.read_exec()
            return self.STAY_FLAG_RET
        elif (cmd_in == ALL_ANGLES):
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
        elif (cmd_in == SET_DFLT_POS):
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

            if (cmd_in == M1_BASE):
                msg = self.cmd.build_cmd_msg(self.cmd.M1_ANGLE, angle)
            elif (cmd_in == M2_SHOULDER):
                msg = self.cmd.build_cmd_msg(self.cmd.M2_ANGLE, angle)
            elif (cmd_in == M3_ELBOW):
                msg = self.cmd.build_cmd_msg(self.cmd.M3_ANGLE, angle)
            elif (cmd_in == M4_WRIST_V):
                msg = self.cmd.build_cmd_msg(self.cmd.M4_ANGLE, angle)
            elif (cmd_in == M5_WRIST_R):
                msg = self.cmd.build_cmd_msg(self.cmd.M5_ANGLE, angle)
            elif (cmd_in == M6_GRIPPER):
                msg = self.cmd.build_cmd_msg(self.cmd.M6_ANGLE, angle)

        self.arduino_serial.write(msg)
        self.print_verbose("\nreading/exec messages from Arduino")
        self.cmd.read_exec()

        # retrieve changed angles from arduino to ensure it matches in the class
        msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
        self.arduino_serial.write(msg)
        self.cmd.read_exec()

        return self.STAY_FLAG_RET

    def kin_menu(self):
        print("\n--- Kinematics functionalities ---\n"
              "1. Rotation Matrix function\n"
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
        while (not digit or not in_range):
            term.clear()
            self.kin_menu()
            read = input("Enter number: ")
            
            digit = read.isdigit() 
            if (digit):
                read = int(read)
                if (read >= KIN_MENU_MIN and read <= KIN_MENU_MAX):
                    in_range = True
                else:
                    print("Invalid input\n")
                    input("-- Press Enter to Continue --")
            else:
                print("Invalid input\n")
                input("-- Press Enter to Continue --")

        if (read == EXIT_VAL_IN):
            return self.EXIT_FLAG_RET
        
        if (read == ROT_MAT_IN): # rotation matrix functionality testing
            term.clear()
            print("--- Testing rotation matrix function ---")

            self.input_current_or_new_angles()
            
            in_range = False
            digit = False
            while (not digit or not in_range):
                print("\nEnter starting frame for rot matrix")
                start_frame = input("Enter starting frame number (0-4): ")

                digit = start_frame.isdigit()
                if (digit):
                    start_frame = int(start_frame)
                    if (start_frame >= START_ROT_IN_MIN and 
                        start_frame <= START_ROT_IN_MAX):
                        in_range = True
                    else:
                        print("Invalid input\n")
                        input("-- Press Enter to Continue --")
                else:
                    print("Invalid input\n")
                    input("-- Press Enter to Continue --")
            
            in_range = False
            digit = False
            while (not digit or not in_range):
                print("\nEnter ending frame for rot matrix")
                end_frame = input("Enter ending frame number (1-5): ")

                digit = end_frame.isdigit()
                if (digit):
                    end_frame = int(end_frame)
                    if (end_frame >= END_ROT_IN_MIN and
                        end_frame <= END_ROT_IN_MAX):
                        in_range = True
                    else:
                        print("Invalid input\n")
                        input("-- Press Enter to Continue --")
                else:
                    print("Invalid input\n")
                    input("-- Press Enter to Continue --")

            rot_matrix = self.kin.create_rot_matrix(start_frame,end_frame)
            print("\n--rotation matrix {:d}_{:d}--".format(start_frame, 
                                                           end_frame))
            print(rot_matrix, "\n")
            
            # reset angles to match braccio
            msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
            self.arduino_serial.write(msg)
            self.cmd.read_exec()
        elif (read == DISP_VECT_IN): # test the displacement vector function
            term.clear()
            print("\n--- Testing displacement vectors ---")
            
            self.input_current_or_new_angles()
            
            # set new displacement vectors and print
            self.kin.create_fill_disp_vects()
            self.kin.print_disp_vects()
            input("-- Press Enter to Continue --")

            # reset angles and displacement vectors
            msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
            self.arduino_serial.write(msg)
            self.cmd.read_exec()
        elif (read == HOMO_TRANS_IN): # Homogeneous transform functionality
            term.clear()
            print("\n-- Testing the Homogeneous Transform Matrix function --")

            self.input_current_or_new_angles()
            self.kin.set_kin_vars() # in case angles changed
            self.kin.print_homo_trans_mats()

            input("-- Press Enter to Continue --")

            # reset angles and displacement vectors
            msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
            self.arduino_serial.write(msg)
            self.cmd.read_exec()

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
        while (not digit or not in_range):
            term.clear()
            print("\nThis section shows the example of a fuzzy logic\n"
                  "controller and print/testing its functionalities\n")
                  
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
                    input("-- Press Enter to Continue --")
            else:
                print("Invalid input\n")
                input("-- Press Enter to Continue --")
         
        if (read == EXIT_VAL):
            return self.EXIT_FLAG_RET
        elif (read == FUZZY_CONT_EX):
            self.fuzzy_con.fuzzy_controller_exec()
        elif (read == PRINT_FUZZY_SETS):
            self.fuzzy_con.print_fuzzy_sets()
        elif (read == MEMBERSHIP_CALC_TEST):
            self.fuzzy_con.membership_test()
        else:
            print("Invalid input\n")
            input("-- Press Enter to Continue --")

        return self.STAY_FLAG_RET
