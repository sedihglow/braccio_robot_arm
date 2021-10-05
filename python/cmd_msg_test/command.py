import struct

class command_msg:
    S0_ANGLE = 0x0 # change servo 1 angle

    # private
    _SHIFT_BYTE = 8
    _CMD_MSG = 2
    _msg = 0

    def __init__(self):
        pass

    def build_cmd_msg(self, cmd, *argv):
        if (cmd == self.S0_ANGLE):
            arg = argv
            self._msg =  struct.pack("bbb", self._CMD_MSG, cmd, arg[0])
        
        return self._msg

class command_interface:
    def __init__(self):
        pass
