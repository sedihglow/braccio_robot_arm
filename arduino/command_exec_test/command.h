#ifndef _COMMAND_H
#define _COMMAND_H

#include <stdint.h>
#include <Arduino.h>

#define S0_ANGLE 0x0

class command {
    public:
        command(Stream &serial); 
        ~command();
        
        void parse_msg(uint8_t *msg);
        void exec_command(uint8_t *msg);

    private:
        Stream &serial;
};

#endif
