from command import command_interface
from arduino_serial import arduino_com

class braccio_interface:
    def __init__(self, verbose, port, baudrate, rtimeout):
        self.verbose = verbose
        self.arduino_serial = arduino_com(port, baudrate, rtimeout)
        self.cmd = command_interface(verbose)

        self.angles = []
        for i in range(0,6,1):
            self.angles.append(0)

    def begin_com(self):
        self.arduino_serial.begin()

        # get setup messages to confirm Arduino is on
        self.read_exec()

        print("finished first read")
        
        # init angles from Arduino
        msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
        self.arduino_serial.write(msg)
        print("request sent")
        self.read_exec()
        print("end final read exec")

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
                    self.cmd.exec_command(p_msg)
                elif(p_msg[0] == self.cmd.FINISH):
                    print("Arduino finished sending message")
                    finished = True

    def print_cmd_menu(self):
        print("Choose angle to set\n"
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
              "2. Invere Kinimatics\n"
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
            pass
        else:
            print("invalid input")
            
        return 0
