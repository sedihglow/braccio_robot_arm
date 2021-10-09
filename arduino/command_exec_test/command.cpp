#include "command.h"

#define PARAM_BUFF 100

command::command(Stream &serial)
{
    serial = serial;
}

command::~command()
{
}

void command::parse_msg(uint8_t *msg)
{
    int i = 0;
    uint8_t msg_type = 0;
    uint8_t cmd = 0;
    uint8_t param[PARAM_BUFF] = {'\0'};

    // parse msg type
    msg_type = msg[0]; 

    // parse cmd type
    cmd = msg[1];
    
    if (cmd == S0_ANGLE) {
        // 1 uint8_t param for S0_ANGLE
        param[0] = msg[2];
    }
}

void command::exec_command(uint8_t *msg)
{
    serial.print("parse done in exec command\n");
    serial.flush();
}
