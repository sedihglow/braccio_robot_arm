from command import command_interface
from arduino_serial import arduino_com
from kin import kinematics
from fuzzy_controller import fuzzy_controller
from image_processing import image_processing
from term import term_utility

class braccio_interface:
    EXIT_FLAG_RET = False # Exit value to exit from menu or program
    STAY_FLAG_RET = True # return value for interfaces staying in program
    NUM_SERVOS = 6

    def __init__(self, verbose, port, baudrate, rtimeout):
        self.term = term_utility(verbose)

        self.arduino_serial = arduino_com(port, baudrate, rtimeout)

        self.kin = kinematics()

        self.cmd = command_interface(self.arduino_serial, self.kin, self.term)

        self.fuzzy_con = fuzzy_controller(self.arduino_serial, self.kin,
                                          self.cmd, self.term)

        self.image_proc = image_processing(self.arduino_serial, self.cmd,
                                           self.kin, self.term)

    # Starts communication with the braccio controller and gets init angles
    def begin_com(self):
        self.term.clear()
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
        IMAGE_PROC_VAL = 4
        EXIT_VAL = 5
        MENU_MIN_IN = 1
        MENU_MAX_IN = 5

        in_range = False
        while (not in_range):
            self.term.clear()
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
                  "4. Image Processing\n"
                  "5. exit")
            read = input("Enter Number: ")

            try:
                read = int(read)
                if (read >= MENU_MIN_IN and read <= MENU_MAX_IN):
                    in_range = True
                else:
                    self.term.input_invalid_wait()
            except ValueError:
                self.term.input_invalid_wait()

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
        elif (read == IMAGE_PROC_VAL): # image processing functionality
            stay_flag = self.STAY_FLAG_RET
            while (stay_flag):
                stay_flag = self.image_processing_interface()
        else:
            self.term.input_invalid_wait()

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
                self.term.clear()
            print("\nUse current Braccio angles or use user input angles?")
            print("1. Current angles.\n"
                  "2. User input angles.")
            read = input("Enter number: ")

            try:
                read = int(read)
                if (read >= MENU_IN_MIN and read <= MENU_IN_MAX):
                    in_range = True
                else:
                    self.term.input_invalid_wait()
            except ValueError:
                self.term.input_invalid_wait()

            first_pass = False

        if (read == USER_ANGLE_VAL):
            self.get_user_angles()
        else: #(read == CURRENT_ANGLES):
            print("Using Braccio's current angles")
            self.term.wait_for_enter()

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
            self.term.clear()
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
            # TODO: See if how the angles are converted and checked with
            # isdigit can be improved using exceptions instead. May not be
            # required but should test more thoroughly to check for any
            # edge cases this implementation might look over.
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
            # TODO: Change from using isdigit to exceptions and maybe check
            # to see if the angle given is between 0 and 180 with an in_range
            # flag instead of digit like other menu implementations in the
            # program. May not be totally required but there may be cases where
            # it handles inputs more cleanly and bug proof
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
        self.term.print_verbose("\nreading/exec messages from Arduino\n")
        self.cmd.read_exec()

        # retrieve changed angles from arduino to ensure it matches in the class
        msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
        self.arduino_serial.write(msg)
        self.cmd.read_exec()

        return self.STAY_FLAG_RET

    # Interface to execute kinematics, forward and inverse.
    def kin_interface(self):
        # menu inputs based on kin_menu()
        ROT_MAT_IN    = 1 # Rotation Matrix Functionality input
        DISP_VECT_IN  = 2 # Displacement Vectors input
        HOMO_TRANS_IN = 3 # Homogeneous Transform Matrix Funcionality input
        INV_KIN_3DOF  = 4
        EXIT_VAL_IN   = 5
        KIN_MENU_MIN  = 1
        KIN_MENU_MAX  = 5
        END_ROT_IN_MIN = 1
        END_ROT_IN_MAX = 5
        START_ROT_IN_MIN = 0
        START_ROT_IN_MAX = 4

        in_range = False
        while (not in_range):
            self.term.clear()
            print("\n--- Kinematics functionalities ---\n"
                  "1. Rotation Matrix function\n"
                  "2. Displacement Vectors\n"
                  "3. Homogeneous Transform Matrix.\n"
                  "4. Inverse Kin 3DOF\n"
                  "5. exit")
            read = input("Enter number: ")

            try:
                read = int(read)
                if (read >= KIN_MENU_MIN and read <= KIN_MENU_MAX):
                    in_range = True
                else:
                    self.term.input_invalid_wait()
            except ValueError:
                self.term.input_invalid_wait()

        if (read == EXIT_VAL_IN):
            return self.EXIT_FLAG_RET

        self.term.clear()
        if (read == ROT_MAT_IN): # rotation matrix functionality testing
            print("--- Testing rotation matrix function ---")

            self.input_current_or_new_angles()

            in_range = False
            while (not in_range):
                print("\nEnter starting frame for rot matrix")
                start_frame = input("Enter starting frame number (0-4): ")

                try:
                    start_frame = int(start_frame)
                    if (start_frame >= START_ROT_IN_MIN and
                        start_frame <= START_ROT_IN_MAX):
                        in_range = True
                    else:
                        self.term.input_invalid_wait()
                except ValueError:
                    self.term.input_invalid_wait()

            in_range = False
            while (not in_range):
                print("\nEnter ending frame for rot matrix")
                end_frame = input("Enter ending frame number (1-5): ")

                try:
                    end_frame = int(end_frame)
                    if (end_frame >= END_ROT_IN_MIN and
                        end_frame <= END_ROT_IN_MAX):
                        in_range = True
                    else:
                        self.term.input_invalid_wait()
                except ValueError:
                    self.term.input_invalid_wait()

            rot_matrix = self.kin.create_rot_matrix(start_frame,end_frame)
            print("\n--rotation matrix {:d}_{:d}--".format(start_frame,
                                                           end_frame))
            print(rot_matrix, "\n")

            self.term.wait_for_enter()

        elif (read == DISP_VECT_IN): # test the displacement vector function
            print("\n--- Testing displacement vectors ---")

            self.input_current_or_new_angles()

            # set new displacement vectors and print
            self.kin.create_fill_disp_vects()
            self.kin.print_disp_vects()

            self.term.wait_for_enter()

        elif (read == HOMO_TRANS_IN): # Homogeneous transform functionality
            print("\n-- Testing the Homogeneous Transform Matrix function --")

            self.input_current_or_new_angles()
            self.kin.set_kin_vars() # in case angles changed
            self.kin.print_homo_trans_mats()

            self.term.wait_for_enter()
        elif (read == INV_KIN_3DOF): # Three degrees of freedom inverse kin
            print("\n-- Demonstrating Inverse Kinematics with 3 DOF --")
            self.kin.three_dof_inverse_kin()
            self.term.wait_for_enter()

        # Make sure the angles in the kin class match the braccio in case
        # the user changed the kin class angles instead of using the current
        # braccio angles.
        msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
        self.arduino_serial.write(msg)
        self.cmd.read_exec()

        self.kin.set_kin_vars()

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
        while (not in_range):
            self.term.clear()
            print("\nThis section shows the example of a fuzzy logic\n"
                  "controller and print/testing its functionalities\n")

            print("1. Fuzzy logic example\n"
                  "2. Print Fuzzy Sets for Braccio\n"
                  "3. Membership calculator testing\n"
                  "4. exit")
            read = input("Enter number: ")

            try:
                read = int(read)
                if (read >= FC_MENU_MIN and read <= FC_MENU_MAX):
                    in_range = True
                else:
                    self.term.input_invalid_wait()
            except ValueError:
                self.term.input_invalid_wait()

        if (read == EXIT_VAL):
            return self.EXIT_FLAG_RET
        elif (read == FUZZY_CONT_EX):
            self.fuzzy_con.fuzzy_controller_exec()

            # Return braccio to default position and update the kinematics class
            msg = self.cmd.build_cmd_msg(self.cmd.SET_DFLT_POS)
            self.arduino_serial.write(msg)
            self.cmd.read_exec()

            # update angles and displacement vectors
            msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
            self.arduino_serial.write(msg)
            self.cmd.read_exec()

            self.kin.set_kin_vars()
        elif (read == PRINT_FUZZY_SETS):
            self.fuzzy_con.print_fuzzy_sets()
        elif (read == MEMBERSHIP_CALC_TEST):
            self.fuzzy_con.membership_test()
        else:
            self.term.input_invalid_wait()


        return self.STAY_FLAG_RET

    def image_processing_interface(self):
        MOTION_NO_BRACCIO_EX = 1 # Motion detection example without Braccio
        MOTION_WITH_BRACCIO_EX = 2 # Motion detection example with Braccio
        EXIT_VAL = 3
        MENU_MIN_VAL = 1
        MENU_MAX_VAL = 3

        in_range = False
        while (not in_range):
            self.term.clear()
            print("\nThis section will demonstrate image processing through\n"
                  "motion detection utilizing opencv functionalities\n")

            print("1. Motion detection without Braccio movement\n"
                  "2. Motion detection with Braccio Movement\n"
                  "3. exit\n")
            read = input("Enter number: ")

            try:
                read = int(read)
                if (read >= MENU_MIN_VAL and read <= MENU_MAX_VAL):
                    in_range = True
                else:
                    self.term.input_invalid_wait()
            except ValueError:
                self.term.input_invalid_wait()

        if (read == EXIT_VAL):
            return self.EXIT_FLAG_RET
        elif (read == MOTION_NO_BRACCIO_EX):
            self.image_proc.webcam_movement_no_braccio()
        elif (read == MOTION_WITH_BRACCIO_EX):
            self.image_proc.webcam_movement_with_braccio()
        else:
            self.term.input_invalid_wait()

        # Input buffer flushes in webcam movement functions but in case it
        # clears the contents and moves on before the arduino sends its
        # finished sending messages and moving the braccio message and
        # something else hits the input buffer we will use read_exec to
        # ensure the buffer gets cleared till finish is recieved.
        #
        # TODO: There may be a chance it hits this spot in the exact moment
        # where the input buffer was flushed, the buffer is empty, in_waiting
        # returns zero, then right after in_waiting returns 0 something hits the
        # input buffer from the arduino because it was mid communication. It is
        # unlikely this will occur with the current implementation but is an
        # edge case to be aware of and there should be a way to ensure this
        # does not cause problems on the off chance it happens.
        #
        # There also may be a chance, but unlikely with the delay between
        # ROI checks in webcam_movement_with_braccio and the input flush at the
        # exit of the webcam reading loop, more than one string of arduino
        # communications may end up in the buffer. This would mean there would
        # be a finish communication from the arduino but its not the end of
        # what is in the buffer. There may have been more than 1 command sent
        # to the arduino and it fills the buffer with more than one string of
        # responses. This would mean there would be more than 1 finish
        # communication from the arduino and read_exec would end after the
        # first one. This senario is resolved with a while loop.
        #
        # UPDATE: Deciding to set braccio to default position and updating the
        # kin class in the image processing class after the functionality that
        # changes the braccios angles.
        # The above situations still apply even though read_exec will
        # be called in the image processing class after the webcam flow is done
        # executing because the read exec in the image processing class may hit
        # a finish that isnt related and come from before the default angle and
        # update angle commands so this while loop still needs to be here to
        # cover that though i do not see it occuring often if at all with how
        # the buffers are cleared through the execution process.
        #
        # Havent decided if doing this while loop in the image processing class
        # woud cover it enough to make this while loop unneccisary but at this
        # current time i do not think i will be doing this while loop in the
        # image processing class and just calling read exec after each command
        # sent like the rest of the program is implemented currently. That may
        # change after some testing if time allows. Either way it shouldnt
        # impact anything important even if it is redundantly checking the
        # input buffer.
        while (self.arduino_serial.arduino.in_waiting > 0):
            self.cmd.read_exec()

        # NOTE: In the current implementaion the kinematics class is updated
        # and handled in the function that changes the braccios position. In
        # further development there may be functions that alter the position
        # and angles of the braccio. If those functionalitys dont update the
        # kin class like image_proc.webcam_movement_with_braccio() then the
        # update of the kin variables need to occur here or in its if
        # statment block similar to other interface functions in this class

        return self.STAY_FLAG_RET
