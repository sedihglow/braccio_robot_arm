from command import command_interface
from arduino_serial import arduino_com
import numpy as np

class braccio_interface:
    def __init__(self, verbose, port, baudrate, rtimeout):
        self.verbose = verbose
        self.arduino_serial = arduino_com(port, baudrate, rtimeout)
        self.cmd = command_interface(verbose)

        self.angles = []
        for i in range(0,6,1):
            self.angles.append(0)
        
        self.link_len = []
        for i in range(0,5,1):
            self.link_len.append(0)
        
        # length in cm
        self.link_len[0] = 6
        self.link_len[1] = 13
        self.link_len[2] = 12
        self.link_len[3] = 6.5
        self.link_len[4] = 12

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
                    print("ACK recieved")
                elif (p_msg[0] == self.cmd.PRINT_MSG):
                    self.cmd.exec_print(p_msg)
                elif(p_msg[0] == self.cmd.CMD_MSG):
                    self.cmd.exec_command(p_msg, self.angles)
                elif(p_msg[0] == self.cmd.FINISH):
                    print("Arduino finished sending message")
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
              "2. Invere Kinematics\n"
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
                exit = self.inverse_kin_interface()
        else:
            print("invalid input")
            
        return 0

    def inverse_kin_menu(self):
        print("Inverse Kinematics functionalities")
        print("1. Rotation Matrix function\n"
              "2. exit")

    # returns a numpy array matrix, on error returns zero matrix
    def create_rot_matrix(self, end_frame):
        # Convert servo angles from degrees to radians
        a0_rad = np.deg2rad(self.angles[0])
        a1_rad = np.deg2rad(self.angles[1])
        a2_rad = np.deg2rad(self.angles[2])
        a3_rad = np.deg2rad(self.angles[3])
        a4_rad = np.deg2rad(self.angles[4])
        a5_rad = np.deg2rad(self.angles[5])
        
        if (end_frame == 0):
            print("ERROR: Invalid start and end frame rot mat, same value")
            error = np.array([[0,0,0],
                              [0,0,0],
                              [0,0,0]])
            return error

        # This matrix helps convert the servo_1 frame to the servo_0 frame.
        rot_mat_0_1 = np.array([[np.cos(a0_rad), 0, np.sin(a0_rad)],
                                [np.sin(a0_rad), 0, -np.cos(a0_rad)],
                                [0, 1, 0]])
        
        if (end_frame == 1):
            return rot_mat_0_1

        # This matrix helps convert the servo_2 frame to the servo_1 frame.
        rot_mat_1_2 = np.array([[np.cos(a1_rad), -np.sin(a1_rad), 0],
                                [np.sin(a1_rad), np.cos(a1_rad), 0],
                                [0, 0, 1]]) 
        if (end_frame == 2):
            rot_mat_0_2 = rot_mat_0_1 @ rot_mat_1_2
            return rot_mat_0_2

        # This matrix helps convert the servo_3 frame to the servo_2 frame.
        rot_mat_2_3 = np.array([[np.cos(a2_rad), -np.sin(a2_rad), 0],
                                [np.sin(a2_rad), np.cos(a2_rad), 0],
                                [0, 0, 1]]) 
 
        if (end_frame == 3):
            rot_mat_0_3 = rot_mat_0_1 @ rot_mat_1_2 @ rot_mat_2_3
            return rot_mat_0_3

        # This matrix helps convert the servo_4 frame to the servo_3 frame.
        rot_mat_3_4 = np.array([[-np.sin(a3_rad), 0, np.cos(a3_rad)],
                                [np.cos(a3_rad), 0, np.sin(a3_rad)],
                                [0, 1, 0]]) 
        
        if (end_frame == 4):
            rot_mat_0_4 = (rot_mat_0_1 @ rot_mat_1_2 @ rot_mat_2_3 @
                           rot_mat_3_4)
            return rot_mat_0_4

        # This matrix helps convert the servo_5 frame to the servo_4 frame.
        rot_mat_4_5 = np.array([[np.cos(a4_rad), -np.sin(a4_rad), 0],
                                [np.sin(a4_rad), np.cos(a4_rad), 0],
                                [0, 0, 1]])
         
        # Calculate the rotation matrix that converts the 
        # end-effector frame (frame 5) to the servo_0 frame.
        rot_mat_0_5 = (rot_mat_0_1 @ rot_mat_1_2 @ rot_mat_2_3 @ rot_mat_3_4 @ 
                       rot_mat_4_5)
        return rot_mat_0_5

    def inverse_kin_interface(self):
        exit_val = 2
        self.inverse_kin_menu()

        read = input("Enter number:")
        read = int(read)

        if (read == exit_val):
            return 1
        
        if (read == 1): # rotation matrix functionality testing
            print("Testing rotation matrix function.")

            print("Enter 6 angles for braccio, servo 0-5 (M1-M6)")
            a1, a2, a3, a4, a5, a6 = input("Enter angles: ").split()
            a1, a2, a3, a4, a5, a6 = [int(a1), int(a2), int(a3), int(a4), 
                                      int(a5), int(a6)]
            self.angles[0] = a1
            self.angles[1] = a2
            self.angles[2] = a3
            self.angles[3] = a4
            self.angles[4] = a5
            self.angles[5] = a6

            print("Enter ending frame for rot matrix")
            end_frame = input("Enter frame number (0-5):")

            rot_matrix = self.create_rot_matrix(end_frame)
            print(rot_matrix)
            
            # reset angles to match braccio
            msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
            self.arduino_serial.write(msg)
            self.read_exec()
            return 0
        return 0
