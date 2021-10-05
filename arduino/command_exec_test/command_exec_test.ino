/*
 * Program to test sending and recieving data from serial.
 * written by: James Ross
 */

#define BAUD_RATE 115200
#define MSG_BUFF 256
#define S_IN_BUFF 10 // serial input buffer size

uint8_t serial_in[S_IN_BUFF] = {'\0'};
char serial_msg[MSG_BUFF] = {'\0'};

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
        sprintf(serial_msg, "serial available, %s\n", serial_in);
        Serial.print(serial_msg);
    }
}
