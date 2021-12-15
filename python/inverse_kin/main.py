from braccio import braccio_interface
import argparse

BAUD_RATE = 115200
SERIAL_PORT = "/dev/ttyACM0"
RTIMEOUT = 0.5 # read serial timeout

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interface to test commands to"
                                                 "arduino through serial")
    parser.add_argument("-v", dest="verbose", default=False, 
                        action='store_true')
    cl_args = parser.parse_args()
    
    braccio = braccio_interface(cl_args.verbose, SERIAL_PORT, 
                                BAUD_RATE, RTIMEOUT)
    braccio.begin_com()

    try:
        exit = 0
        while (not exit):
            exit = braccio.interface_director()
    except KeyboardInterrupt:
        print("exiting...")
    finally:
        print("exiting...")
        

