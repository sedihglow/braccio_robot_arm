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

    def eprint(*args, **kwargs):
        print(*args, file=sys.stderr, **kwargs)

    def wait_for_enter(self):
        input("--- Press Enter to Continue ---")

    def input_invalid_wait(self):
        print("Invalid input\n")
        self.wait_for_enter()
