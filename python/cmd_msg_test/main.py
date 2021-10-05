from arduino_serial import arduino_com
from command import command_msg
from command import command_interface

BAUD_RATE = 115200
SERIAL_PORT = "/dev/ttyACM0"
RTIMEOUT = 0.5 # read serial timeout

if __name__ == "__main__":
    command = command_interface()
    cmd_msg = command_msg()
    arduino_serial = arduino_com(SERIAL_PORT, BAUD_RATE, RTIMEOUT)
    arduino_serial.begin()

    msg = cmd_msg.build_cmd_msg(cmd_msg.S0_ANGLE, 90) 

    arduino_serial.write(msg)
    print(msg)
    print(arduino_serial.read_line())

