import os
import sys

class term_utility:
    def __init__(self, verbose):
        self.verbose = verbose

    def clear(self):
       os.system("clear")

    def sys_print(self, msg):
        sys.stdout.write(msg)

    def print_verbose(self, msg):
        if (self.verbose):
            self.sys_print(msg)

    def check_verbose(self):
        return self.verbose
