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

    def read(self, delay=0.1):
        reading = self.arduino.read()
        time.sleep(delay)
        return reading

    def read_line(self, delay=0.1):
        reading = self.arduino.readline()
        time.sleep(delay)
        return reading
