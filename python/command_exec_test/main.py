from arduino_serial import arduino_com
from command import command_interface
import argparse
import concurrent.futures

BAUD_RATE = 115200
SERIAL_PORT = "/dev/ttyACM0"
RTIMEOUT = 0.5 # read serial timeout

def read_exec(cmd, arduino_serial):
    read = arduino_serial.read()
    if (read):
        msg_size = int.from_bytes(read, byteorder="little", signed=False)
        read = arduino_serial.read(msg_size)
        p_msg = cmd.parse_in_msg(read)
        if (p_msg[0] == cmd.ACK):
            print("ACK recieved")
            return 0;
        elif (p_msg[0] == cmd.PRINT_MSG):
            cmd.exec_print(p_msg)
            return 0;
        elif(p_msg[0] == cmd.CMD_MSG):
            cmd.exec_command(p_msg)
            return 0;
        elif(p_msg[0] == cmd.FINISH):
            print("Arduino finished sending message");
            return 1;

def print_cmd_menu():
    print("Choose angle to set\n"
          "1. m1, base\n"
          "2. m2, shoulder\n"
          "3. m3, elbow\n"
          "4. m4, wrist vertical\n"
          "5. m5, write rotation\n"
          "6. m6, gripper\n"
          "7. All angles\n"
          "8. Request all angles\n"
          "9. exit")

def user_input(cmd, arduino_serial):
    print_cmd_menu()
    change_angle = input("Enter number: ")
    change_angle = int(change_angle)

    if (change_angle > 9 or change_angle < 1):
        print("Invalid Input")
        return 0

    if (change_angle == 9):
            print("exit program")
            return 1
    
    if (change_angle == 7):
        a1, a2, a3, a4, a5, a6 = input("Enter angles: ").split()
        a1, a2, a3, a4, a5, a6 = [int(a1), int(a2), int(a3), int(a4), int(a5),
                                  int(a6)]
        msg = cmd.build_cmd_msg(cmd.MX_ANGLE, a1, a2, a3, a4, a5, a6)
        arduino_serial.write(msg)
    elif (change_angle == 9):
        msg = cmd.buid_cmd_msg(cmd.REQUEST_MX_ANGLE)
    else:
        angle = input("Enter angle: ")
        angle = int(angle)
        if (change_angle == 1):
            msg = cmd.build_cmd_msg(cmd.M1_ANGLE, angle)
        elif (change_angle == 2):
            msg = cmd.build_cmd_msg(cmd.M2_ANGLE, angle)
        elif (change_angle == 3):
            msg = cmd.build_cmd_msg(cmd.M3_ANGLE, angle)
        elif (change_angle == 4):
            msg = cmd.build_cmd_msg(cmd.M4_ANGLE, angle)
        elif (change_angle == 5):
            msg = cmd.build_cmd_msg(cmd.M5_ANGLE, angle)
        elif (change_angle == 6):
            msg = cmd.build_cmd_msg(cmd.M6_ANGLE, angle)

        arduino_serial.write(msg)
    return 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interface to test commands to"
                                                 "arduino through serial")
    parser.add_argument("-v", dest="verbose", default=False, 
                        action='store_true')
    cl_args = parser.parse_args()

    arduino_serial = arduino_com(SERIAL_PORT, BAUD_RATE, RTIMEOUT)
    arduino_serial.begin()
    cmd = command_interface(cl_args.verbose)

    try:
        exit = 0
        while (not exit):
            exit = user_input(cmd, arduino_serial)

            print("reading messages from arduino")
            finished = 0
            while (not finished and not exit):
                finished = read_exec(cmd, arduino_serial)
    except KeyboardInterrupt:
        print("exiting...")
    finally:
        print("exiting...")
        

