from arduino_serial import arduino_com
from command import command_interface

import time

BAUD_RATE = 115200
SERIAL_PORT = "/dev/ttyACM0"
RTIMEOUT = 0.5 # read serial timeout

if __name__ == "__main__":
    arduino_serial = arduino_com(SERIAL_PORT, BAUD_RATE, RTIMEOUT)
    arduino_serial.begin()
    cmd = command_interface()

    msg = cmd.build_cmd_msg(cmd.M1_ANGLE, 90) 

    arduino_serial.write(msg)
    print("reading message from arduino")

    for i in range(0,6,1):
        print("read {}".format(i))
        data = arduino_serial.read_line()
        print(data)
        if (data):
            cmd.parse_in_msg(data)
