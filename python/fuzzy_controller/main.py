from braccio import braccio_interface
import argparse

BAUD_RATE = 115200
SERIAL_PORT = "/dev/ttyACM0"
RTIMEOUT = 0.5 # read serial timeout

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interface to control and test"
                                                 " funcinality of host and"
                                                 " Ardunio controller.")
    parser.add_argument("-v", dest="verbose", default=False, 
                        action='store_true')
    parser.add_argument("-p", dest="port", default=SERIAL_PORT)
    cl_args = parser.parse_args()
    
    braccio = braccio_interface(cl_args.verbose, cl_args.port, 
                                BAUD_RATE, RTIMEOUT)
    braccio.begin_com()

    try:
        stay_flag = True
        while (stay_flag):
            stay_flag = braccio.interface_director()
    except KeyboardInterrupt:
        print("\nexiting...")
    finally:
        print("\nexiting...")
        

