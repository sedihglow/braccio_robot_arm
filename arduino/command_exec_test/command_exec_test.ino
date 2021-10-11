/*
 * Program to test sending and recieving data from serial.
 * written by: James Ross
 */

#include "command.h"

#define BAUD_RATE 115200
#define MSG_BUFF 256
#define S_IN_BUFF 10 // serial input buffer size

uint8_t serial_in[S_IN_BUFF] = {'\0'};
char serial_msg[MSG_BUFF] = {'\0'};

// initialize cmd class with serial
command cmd = command(Serial);

void setup()
{
    Serial.begin(BAUD_RATE);
    while (!Serial) {} // wait for serial interace to connect
    Serial.setTimeout(1);
    Serial.print("Setup Complete"); // let serial com know setup complete
}

void loop()
{
    int i = 0;
    int num_recv = 0;
    if (Serial.available()) {
        num_recv = Serial.readBytes(serial_in, S_IN_BUFF-1); // room for '\0'
        serial_in[num_recv] = '\0';

        // Below two lines work fine with python code
        sprintf(serial_msg, "serial available, %s\n", serial_in);
        Serial.print(serial_msg);

        /* printing in class does not work. Behavior makes me see 
         * "setup complete" each read from the setup function for some reason.
         * (when above two lines are commented out)
         */
        //cmd.exec_command(serial_in);
        
        /* if above 3 lines are removed and line below is uncommented, i can
         * read it multiple times from the python code.
         */
        //Serial.println("in loop");
    }
}
