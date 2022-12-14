import serial
import time

# Communicates with the arduino controller through the serial interface
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

    def write(self, msg, delay=0.05):
        self.arduino.write(msg)
        time.sleep(delay)

    def read(self, size=1, delay=0.1):
        reading = self.arduino.read(size)
        time.sleep(delay)
        return reading

    def read_line(self, delay=0.1):
        reading = self.arduino.readline()
        time.sleep(delay)
        return reading

    def clear_input_buffer(self):
        if (self.arduino.in_waiting > 0):
            self.arduino.reset_input_buffer()
