import serial
import time

class arduino_com:

    def __init__(self, port, baudrate, rtimeout):
        self.port = port
        self.baudrate = baudrate
        self.rtimeout = rtimeout
        self.arduino = 0 # will be serial object

    def begin(self):
        self.arduino = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.rtimeout)

    def write(self, msg, delay=0.05):
        self.arduino.write(msg.encode()) 
        time.sleep(delay)

    def read(self):
        return self.arduino.read()

    def read_line(self):
        data = self.arduino.readline()
        return data.decode()
