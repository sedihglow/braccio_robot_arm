import struct

class command_msg:
    CMD_MSG = 0x0
    M1_ANGLE = 0x1 # change servo 1 angle

    def __init__(self):
        self.read_delay = 0
        self._msg = 0

    def build_cmd_msg(self, cmd, *argv):
        if (cmd == self.M1_ANGLE):
            arg = argv
            self._msg =  struct.pack("bbbb", self.CMD_MSG, cmd, 1, arg[0])
        
        return self._msg
