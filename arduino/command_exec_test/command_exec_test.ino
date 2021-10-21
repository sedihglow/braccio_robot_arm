/*
 * Program to test sending and recieving data from serial.
 * written by: James Ross
 */

#include "braccio_arm.h"

#define BAUD_RATE 115200
#define MSG_BUFF 256
#define S_IN_BUFF 10 // serial input buffer size

uint8_t serial_in[S_IN_BUFF] = {'\0'};
char serial_msg[MSG_BUFF] = {'\0'};

braccio_arm braccio = braccio_arm(Serial);

void setup()
{
    Serial.begin(BAUD_RATE);
    while (!Serial) {} // wait for serial interace to connect
    Serial.print("Setup Complete\n"); // let serial com know setup complete
}

void loop()
{
    int num_recv = 0;
    parsed_msg_s parsed_msg;
    if (Serial.available()) {
        num_recv = Serial.readBytes(serial_in, S_IN_BUFF-1); // room for '\0'
        serial_in[num_recv] = '\0';
        braccio.parse_msg(serial_in, &parsed_msg);
        braccio.exec_command(&parsed_msg);
        braccio.send_verbose("test 1");
    }
}
