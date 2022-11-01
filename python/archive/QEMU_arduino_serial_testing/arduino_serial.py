import serial
import time

class arduino_com:

    def __init__(self, port, baudrate, rtimeout):
        self.port = port
        self.baudrate = baudrate
        self.rtimeout = rtimeout
        self.arduino = 0 # will be serial object

    def begin(self, delay=2):
        self.arduino = serial.Serial(port=self.port, baudrate=self.baudrate, 
                                     timeout=self.rtimeout)
        # arduino resets after serial connection, wait for arduino to setup 
        time.sleep(delay)
        while (not self.read_line()):
            pass

    def write(self, msg, delay=0.05):
        self.arduino.write(msg) 
        time.sleep(delay)

    def read(self):
        return self.arduino.read()

    def read_line(self):
        return self.arduino.readline()
