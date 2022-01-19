from command import command_interface
from arduino_serial import arduino_com
from kin import kinematics

class braccio_interface:
    def __init__(self, verbose, port, baudrate, rtimeout):
        self.verbose = verbose
        self.arduino_serial = arduino_com(port, baudrate, rtimeout)
        self.cmd = command_interface(verbose)

        self.kin = kinematics()
    
    def verbose_print(self, msg):
        if (self.verbose):
            print(msg)
         
    def begin_com(self):
        self.arduino_serial.begin()

        # get setup messages to confirm Arduino is on
        self.read_exec()

        # init angles from Arduino
        msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
        self.arduino_serial.write(msg)
        self.read_exec()

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
                    self.set_kin_vars()
                elif(p_msg[0] == self.cmd.FINISH):
                    self.verbose_print("Arduino finished sending message")
                    finished = True

    def print_cmd_menu(self):
        print("Choose angle to set or command to send\n"
              "1. m1, base\n"
              "2. m2, shoulder\n"
              "3. m3, elbow\n"
              "4. m4, wrist vertical\n"
              "5. m5, write rotation\n"
              "6. m6, gripper\n"
              "7. All angles\n"
              "8. Request all angles\n"
              "9. Set default position\n"
              "10. exit")

    def cmd_menu_input_send(self):
        self.print_cmd_menu()
        change_angle = input("Enter number: ")
        change_angle = int(change_angle)

        if (change_angle > 10 or change_angle < 1):
            print("Invalid Input")
            return 0

        if (change_angle == 10):
                print("exit program")
                return 1
        
        if (change_angle == 7):
            a1, a2, a3, a4, a5, a6 = input("Enter angles: ").split()
            a1, a2, a3, a4, a5, a6 = [int(a1), int(a2), int(a3), int(a4), 
                                      int(a5), int(a6)]
            msg = self.cmd.build_cmd_msg(self.cmd.MX_ANGLE, a1, a2, a3, a4, a5, 
                                         a6)
        elif (change_angle == 8):
            msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
            self.arduino_serial.write(msg)
            return 0
        elif (change_angle == 9):
            msg = self.cmd.build_cmd_msg(self.cmd.SET_DFLT_POS)
        else:
            angle = input("Enter angle: ")
            angle = int(angle)
            if (change_angle == 1):
                msg = self.cmd.build_cmd_msg(self.cmd.M1_ANGLE, angle)
            elif (change_angle == 2):
                msg = self.cmd.build_cmd_msg(self.cmd.M2_ANGLE, angle)
            elif (change_angle == 3):
                msg = self.cmd.build_cmd_msg(self.cmd.M3_ANGLE, angle)
            elif (change_angle == 4):
                msg = self.cmd.build_cmd_msg(self.cmd.M4_ANGLE, angle)
            elif (change_angle == 5):
                msg = self.cmd.build_cmd_msg(self.cmd.M5_ANGLE, angle)
            elif (change_angle == 6):
                msg = self.cmd.build_cmd_msg(self.cmd.M6_ANGLE, angle)

        self.arduino_serial.write(msg)

        # retreive changed angles from arduino to ensure it matches in the class
        msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
        self.arduino_serial.write(msg)
        return 0

    def interface_director(self):
        exit_val = 4
        print("This program will allow you to tinker with all the servos via\n"
              "the command interface with the Arduino controller.\n"
              "It will also showcase an implementation of inverse kinimatics\n"
              "for this Braccio robot arm")

        # print menu for cmd or inverse kin
        print("Which functionality would you like to run?") 
        print("1. Command Interface (Tinker with servos and commands)\n"
              "2. Kinematics\n"
              "4. exit")
        read = input("Enter Number: ")
        read = int(read)

        if (read == exit_val):
            return 1

        if (read == 1): # cmd interface
            exit = False
            while (not exit):
                exit = self.cmd_menu_input_send()

                print("reading messages from Arduino")
                if (not exit):
                    self.read_exec()
        elif (read == 2): # inverse kin
            exit = False
            while (not exit):
                exit = self.kin_interface()
        else:
            print("invalid input")
            
        return 0

    def kin_menu(self):
        print("Kinematics functionalities")
        print("1. Rotation Matrix function\n"
              "2. Displacement Vectors\n"
              "3. Homogeneous Transform Matrix.\n"
              "4. exit")
    
    # fills kin.angles with user input
    def get_user_angles(self):
        print("Enter 6 angles for braccio, servo 0-5 (M1-M6)")
        a1, a2, a3, a4, a5, a6 = input("Enter angles: ").split()
        a1, a2, a3, a4, a5, a6 = [int(a1), int(a2), int(a3), int(a4), 
                                  int(a5), int(a6)]
        self.kin.angles[0] = a1
        self.kin.angles[1] = a2
        self.kin.angles[2] = a3
        self.kin.angles[3] = a4
        self.kin.angles[4] = a5
        self.kin.angles[5] = a6

    def input_current_or_new_angles(self):
            print("Use current Braccio angles or use user input angles?")
            print("1. Current angles.\n"
                  "2. User input angles.")
            read = input("Enter number: ")
            read = int(read)

            if (read == 2):
                self.get_user_angles()

    # set kinematic variables with angles that are set
    def set_kin_vars(self): 
        self.kin.create_rot_matrix()
        self.kin.create_fill_disp_vects()
        self.kin.create_homo_trans()
    
    def print_homo_trans_mats(self):
        print("--Homogeneous transform 0_1--")
        print(self.kin.homo_trans_mat[0])
        print("--Homogeneous transform 1_2--")
        print(self.kin.homo_trans_mat[1])
        print("--Homogeneous transform 2_3--")
        print(self.kin.homo_trans_mat[2])
        print("--Homogeneous transform 3_4--")
        print(self.kin.homo_trans_mat[3])
        print("--Homogeneous transform 4_5--")
        print(self.kin.homo_trans_mat[4])
        print("--Homogeneous transform 0_5--")
        print(self.kin.homo_trans_mat[5])

    def kin_interface(self):
        exit_val = 4
        self.kin_menu()

        read = input("Enter number: ")
        read = int(read)

        if (read == exit_val):
            return 1
        
        if (read == 1): # rotation matrix functionality testing
            print("Testing rotation matrix function.")
            
            self.input_current_or_new_angles()

            print("Enter starting frame for rot matrix")
            start_frame = input("Enter frame number (0-4)")
            start_frame = int(start_frame)

            print("Enter ending frame for rot matrix")
            end_frame = input("Enter frame number (0-5): ")
            end_frame = int(end_frame)

            rot_matrix = self.kin.create_rot_matrix(start_frame,end_frame)
            print("--rotation matrix {:d}_{:d}--".format(start_frame, end_frame))
            print(rot_matrix)
            
            # reset angles to match braccio
            msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
            self.arduino_serial.write(msg)
            self.read_exec()
            return 0
        elif (read == 2): # test the displacement vector function
            print("Testing displacement vectors")
            
            self.input_current_or_new_angles()
            
            # set new displacement vectors and print
            self.kin.create_fill_disp_vects()
            self.kin.print_disp_vects()

            # reset angles and displacement vectors
            msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
            self.arduino_serial.write(msg)
            self.read_exec()
            self.set_kin_vars()
            return 0
        elif (read == 3):
            print("Testing the Homogeneous Transform Matrix functionality.")
            self.input_current_or_new_angles()
            self.set_kin_vars() # in case angles changed
            self.print_homo_trans_mats()
            return 0

        return 0

