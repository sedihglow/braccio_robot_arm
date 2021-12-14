import struct
import sys

class command_interface:
    # message types
    CMD_MSG = 0x0
    PRINT_MSG = 0x1
    ACK = 0x2
    FINISH = 0x3

    # incomming message command
    PRINT_GENERAL = 0x0
    PRINT_ERROR = 0x1
    PRINT_VERBOSE = 0x2
    SEND_ANGLES = 0x3

    # outgoing message command
    M1_ANGLE = 0x1
    M2_ANGLE = 0x2
    M3_ANGLE = 0x3
    M4_ANGLE = 0x4
    M5_ANGLE = 0x5
    M6_ANGLE = 0x6
    MX_ANGLE = 0x7
    REQUEST_MX_ANGLE = 0x8
    SET_DFLT_POS = 0x9
    
    def __init__(self, verbose):
        self.verbose = verbose 
    
    def sys_print(self, msg):
        sys.stdout.write(msg)

    def print_verbose(self, msg):
        if (self.verbose):
            self.sys_print(msg) # prints the string without adding '\n'
        
    def build_cmd_msg(self, cmd, *argv):
        msg = 0
        arg = argv

        if (cmd == self.MX_ANGLE):
            msg = struct.pack("9B", self.CMD_MSG, cmd, 6, arg[0],
                              arg[1], arg[2], arg[3], arg[4], arg[5])
        elif (cmd == self.M1_ANGLE or cmd == self.M2_ANGLE or 
              cmd == self.M3_ANGLE or cmd == self.M4_ANGLE or
              cmd == self.M5_ANGLE or cmd == self.M6_ANGLE):
            msg =  struct.pack("4B", self.CMD_MSG, cmd, 1, arg[0])
        elif (cmd == self.REQUEST_MX_ANGLE):
            msg = struct.pack("3B", self.CMD_MSG, cmd, 0)
        elif (cmd == self.SET_DFLT_POS):
            msg = struct.pack("3B", self.CMD_MSG, cmd, 0)
        
        return msg

    def parse_in_msg(self, msg):
        parsed = 0
        
        parsed = struct.unpack("B", msg[0:1])
        if (parsed[0] == self.ACK):
            return parsed
        elif (parsed[0] == self.FINISH):
            return parsed
        
        parsed = struct.unpack("3B", msg[:3])
        
        if (parsed[0] == self.PRINT_MSG):
            parsed = struct.unpack("3B{}s".format(parsed[2]), msg)
        elif(parsed[0] == self.CMD_MSG):
            parsed = struct.unpack("3B{}B".format(parsed[2]), msg)

        return parsed

    def exec_command(self, p_msg):
        i = 0
        param = []

        msg_type = p_msg[i]
        i += 1

        if (msg_type != self.CMD_MSG):
            print("ERROR: not command message in exec_command")
            return

        cmd = p_msg[i]
        i += 1
        
        param_len = p_msg[i]
        i += 1

        for k in range(0,param_len):
            param.append(p_msg[i])
            i += 1

        if (cmd == self.SEND_ANGLES):
            print(param)

    def exec_print(self, p_msg):
        i = 0

        msg_type = p_msg[i]
        if (msg_type != self.PRINT_MSG):
            print("ERROR: not print message in exec_print\n")
            return

        i += 1
        cmd = p_msg[i]

        i += 1
        param_len = p_msg[i] # includes '\n'

        i += 1
        to_print = p_msg[i] # include '\n'
        
        if (cmd == self.PRINT_GENERAL):
            self.sys_print(to_print.decode())
        elif (cmd == self.PRINT_ERROR):
            self.sys_print("BOARD ERROR: {}".format(to_print.decode()))
        elif (cmd == self.PRINT_VERBOSE):
            self.print_verbose(to_print.decode())


