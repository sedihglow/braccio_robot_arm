# NOTE: read_exec is outdated and changed in main.py that is not threaded. It
# will need to be migrated to here after changed to the message format from the
# arduino

from arduino_serial import arduino_com
from command import command_interface
import argparse
import concurrent.futures

BAUD_RATE = 115200
SERIAL_PORT = "/dev/ttyACM0"
RTIMEOUT = 0.5 # read serial timeout

def read_exec(cmd, arduino_serial):
    print("reading messages from arduino")
    while (True):
        read = arduino_serial.read_line()
        if (read):
            p_msg = cmd.parse_in_msg(read)
            if (p_msg[0] == cmd.ACK):
                print("ACK recieved")
            elif (p_msg[0] == cmd.PRINT_MSG):
                cmd.exec_print(p_msg)
            elif(p_msg[0] == cmd.CMD_MSG):
                cmd.exec_command(p_msg)

def print_cmd_menu():
    print("Choose angle to set\n"
          "1. m1, base\n"
          "2. m2, shoulder\n"
          "3. m3, elbow\n"
          "4. m4, wrist vertical\n"
          "5. m5, write rotation\n"
          "6. m6, gripper\n"
          "7. All angles")

def user_input(cmd, arduino_serial):
    while (True):
        print_cmd_menu()
        change_angle = input("Enter number: ")
        change_angle = int(change_angle)

        if (change_angle > 7 or change_angle < 1):
            print("Invalid Input")
            continue 
        
        if (change_angle == 7):
            a1, a2, a3, a4, a5, a6 = input("Enter angles: ").split()
            a1, a2, a3, a4, a5, a6 = [int(a1), int(a2), int(a3), int(a4), int(a5),
                                      int(a6)]
            #print("%d, %d, %d, %d, %d, %d" % (a1, a2, a3, a4, a5, a6))
            msg = cmd.build_cmd_msg(cmd.MX_ANGLE, a1, a2, a3, a4, a5, a6)
            print(msg)
            arduino_serial.write(msg)
        else:
            angle = input("Enter angle: ")
            angle = int(angle)
            #print(angle)
            msg = cmd.build_cmd_msg(cmd.M1_ANGLE, angle)
            print(msg)
            arduino_serial.write(msg)
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interface to test commands to"
                                                 "arduino through serial")
    parser.add_argument("-v", dest="verbose", default=False, 
                        action='store_true')
    cl_args = parser.parse_args()

    arduino_serial = arduino_com(SERIAL_PORT, BAUD_RATE, RTIMEOUT)
    arduino_serial.begin()
    cmd = command_interface(cl_args.verbose)


    with concurrent.futures.ThreadPoolExecutor() as executor:
        # Map background tasks to string representing event
        futures = {
            executor.submit(read_exec, cmd, arduino_serial): "read_exec",
            executor.submit(user_input, cmd, arduino_serial): "user_input",
        }
        try:
            while(True):
                for future in concurrent.futures.as_completed(futures):
                    event = futures[future]
                    del futures[future]
                    error = future.exception()
                    if error is not None:
                        # Raise on error
                        print('%r generated an exception: %s' % (event, exc))
                        raise error
                    # Get result of function on success
                    data = future.result()
                    #if event == "user_input":
                        # Ask user for more input
                        #futures[executor.submit(user_input, cmd, arduino_serial)] = "user_input"
                
        finally:
            for future in futures:
                if not future.done():
                    future.cancel()
                else:
                    # For futures which are done but have exceptions which we didn't
                    # raise, collect their exceptions
                    future.exception()
