from arduino_serial import arduino_com
from command import command_interface

import time

BAUD_RATE = 115200
SERIAL_PORT = "/dev/ttyACM0"
RTIMEOUT = 0.5 # read serial timeout

if __name__ == "__main__":
    arduino_serial = arduino_com(SERIAL_PORT, BAUD_RATE, RTIMEOUT)
    arduino_serial.begin()
    cmd = command_interface(True)

    msg = cmd.build_cmd_msg(cmd.MX_ANGLE, 45, 45, 45, 45, 45, 45) 

    arduino_serial.write(msg)
    print("reading messages from arduino")
   
    while (True):
        data = arduino_serial.read_line()
        if (data):
            p_msg = cmd.parse_in_msg(data)
            if (p_msg[0] == cmd.ACK):
                continue
            elif (p_msg[0] == cmd.PRINT_MSG):
                cmd.exec_print(p_msg)
            elif(p_msg[0] == cmd.CMD_MSG):
                cmd.exec_command(p_msg)
