/*
 * Program to control the Braccio robot arm with commands from the host
 * computer over a serial interface.
 *
 * Originally written on an Arduino UNO but space ran out with full buffer
 * sizes. It may still fit on an UNO with smaller buffer sizes but is now
 * being controlled with an Arduino DUE.
 *
 * written by: James Ross
 */

#include "braccio_arm.h"

#define BAUD_RATE 115200
#define S_IN_BUFF 20 // serial input buffer size

uint8_t serial_in[S_IN_BUFF] = {'\0'};

braccio_arm braccio = braccio_arm(Serial);

void setup()
{ 
    // Serial.begin must be here since stream object doesnt contain begin()
    Serial.begin(BAUD_RATE);
    while (!Serial) {} // wait for serial interace to connect
    
    // initialize the robot arm, goes to default position
    braccio.init_arm();
    
    braccio.send_verbose("Setup Complete\n");
    braccio.send_finish();
}

void loop()
{
    parsed_msg_s parsed_msg;
    if (braccio.serial_avail()){
        braccio.serial_read(serial_in, S_IN_BUFF);
        braccio.send_ack();
        braccio.parse_msg(serial_in, &parsed_msg);
        braccio.exec_command(&parsed_msg);
        braccio.send_finish();
    }
}
