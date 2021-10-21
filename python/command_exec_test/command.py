import struct

class command_interface:
    # message types
    CMD_MSG = 0x0
    PRINT_MSG = 0x1

    # incomming message command
    PRINT_GENERAL = 0x0
    PRINT_ERROR = 0x1
    PRINT_VERBOSE = 0x2

    # outgoing message command
    M1_ANGLE = 0x1
    M2_ANGLE = 0x2
    M3_ANGLE = 0x3
    M4_ANGLE = 0x4
    M5_ANGLE = 0x5
    M6_ANGLE = 0x6
    MX_ANGLE = 0x7

    def __init__(self):
        pass
        
    def build_cmd_msg(self, cmd, *argv):
        msg = 0
        arg = argv

        if (cmd == self.MX_ANGLE):
            msg = struct.pack("9b", self.CMD_MSG, cmd, 6, arg[0],
                              arg[1], arg[2], arg[3], arg[4], arg[5])
        elif (cmd == self.M1_ANGLE or cmd == self.M2_ANGLE or 
              cmd == self.M3_ANGLE or cmd == self.M4_ANGLE or
              cmd == self.M5_ANGLE or cmd == self.M6_ANGLE):
            msg =  struct.pack("4b", self.CMD_MSG, cmd, 1, arg[0])
        
        return msg

    def parse_in_msg(self, msg):
        pass 
             
