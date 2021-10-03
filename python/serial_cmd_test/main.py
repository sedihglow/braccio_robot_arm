from arduino_serial import arduino_com
import serial
import time

BAUD_RATE = 115200
SERIAL_PORT = "/dev/ttyACM0"
RTIMEOUT = 0.5 # read serial timeout

if __name__ == "__main__":

    while True:
        arduino_serial = arduino_com(SERIAL_PORT, BAUD_RATE, RTIMEOUT)
        arduino_serial.begin()

        print("enter message: ", end='')
        var = input()
        print ("you entered", var)
        arduino_serial.write(var)
        data = arduino_serial.read_line()
        print(data)
