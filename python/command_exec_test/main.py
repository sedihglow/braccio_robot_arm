from arduino_serial import arduino_com
from command import command_msg

import time

BAUD_RATE = 115200
SERIAL_PORT = "/dev/ttyACM0"
RTIMEOUT = 0.5 # read serial timeout

if __name__ == "__main__":
    cmd_msg = command_msg()
    arduino_serial = arduino_com(SERIAL_PORT, BAUD_RATE, RTIMEOUT)
    arduino_serial.begin()

    msg = cmd_msg.build_cmd_msg(cmd_msg.M1_ANGLE, 90) 

    arduino_serial.write(msg)
    print("reading message from arduino")

    data = arduino_serial.read_line()
    print(data)

    data = arduino_serial.read_line()
    print(data)

    data = arduino_serial.read_line()
    print(data)
