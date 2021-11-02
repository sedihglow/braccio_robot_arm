/*
 * Program to test sending and recieving data from serial going through
 * a command interface with braccio robot arm
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

    Serial.print("Setup Complete\n"); // let serial com know setup complete
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
