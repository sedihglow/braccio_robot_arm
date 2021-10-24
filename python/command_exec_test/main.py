from arduino_serial import arduino_com
from command import command_interface
import argparse
import concurrent.futures

BAUD_RATE = 115200
SERIAL_PORT = "/dev/ttyACM0"
RTIMEOUT = 0.5 # read serial timeout

def read_exec(cmd, arduino_serial):
    print("reading messages from arduino")
    while (True):
        read = arduino_serial.read_line()
        if (read):
            p_msg = cmd.parse_in_msg(read)
            if (p_msg[0] == cmd.ACK):
                print("ACK recieved")
            elif (p_msg[0] == cmd.PRINT_MSG):
                cmd.exec_print(p_msg)
            elif(p_msg[0] == cmd.CMD_MSG):
                cmd.exec_command(p_msg)

def print_cmd_menu():
    print("Choose angle to set\n"
          "1. m1, base\n"
          "2. m2, shoulder\n"
          "3. m3, elbow\n"
          "4. m4, wrist vertical\n"
          "5. m5, write rotation\n"
          "6. m6, gripper\n"
          "7. All angles")
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interface to test commands to"
                                                 "arduino through serial")
    parser.add_argument("-v", dest="verbose", default=False, 
                        action='store_true')
    cl_args = parser.parse_args()

    arduino_serial = arduino_com(SERIAL_PORT, BAUD_RATE, RTIMEOUT)
    arduino_serial.begin()
    cmd = command_interface(cl_args.verbose)

    msg = cmd.build_cmd_msg(cmd.MX_ANGLE, 45, 45, 10, 45, 45, 45) 

    arduino_serial.write(msg)

    with concurrent.futures.ThreadPoolExecutor() as executor:
        future = executor.submit(read_exec, cmd, arduino_serial)
        print_cmd_menu()
        change_angle = input("Enter number: ")
        change_angle = int(change_angle)
        print(change_angle)

