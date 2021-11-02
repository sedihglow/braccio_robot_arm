#ifndef _BRACCIO_ARM_H
#define _BRACCIO_ARM_H

#include <Arduino.h>
#include <stdint.h>
#include <errno.h>
#include <Braccio.h>
#include <Servo.h>

#define DFLT_STEP_DELAY 20 // ms

#define MSG_SIZE_NO_PARAM 3 // msg_type, cmd, param_len, 3 bytes
#define PARAM_BUFF 250
#define IO_MSG_BUFF (MSG_SIZE_NO_PARAM + PARAM_BUFF)

#define PARAM_STRLEN 0

// msg type definitions
#define CMD_MSG 0x0
#define PRINT_MSG 0x1
#define ACK 0x2
#define FINISH 0x3

// outgoing cmd definitions
#define PRINT_GENERAL 0x0
#define PRINT_ERROR 0x1
#define PRINT_VERBOSE 0x2

// min max angles
#define M1_MIN_ANGLE 0
#define M1_MAX_ANGLE 180
#define M2_MIN_ANGLE 15
#define M2_MAX_ANGLE 165
#define M3_MIN_ANGLE 0
#define M3_MAX_ANGLE 180
#define M4_MIN_ANGLE 0
#define M4_MAX_ANGLE 180
#define M5_MIN_ANGLE 0
#define M5_MAX_ANGLE 180
#define M6_MIN_ANGLE 10 // tongue is open
#define M6_MAX_ANGLE 73 // gripper is closed

// Safety angles
#define M1_SAFE_ANGLE 90
#define M2_SAFE_ANGLE 45
#define M3_SAFE_ANGLE 180
#define M4_SAFE_ANGLE 180
#define M5_SAFE_ANGLE 90
#define M6_SAFE_ANGLE 10

enum servo_angle_cmd {
    M1_ANGLE=1, // base
    M2_ANGLE,   // shoulder
    M3_ANGLE,   // elbow
    M4_ANGLE,   // wrist vertial
    M5_ANGLE,   // wrist rotation
    M6_ANGLE,   // gripper
    MX_ANGLE    // braccio_arm for all servo angles at once
};


typedef struct parsed_io_msg {
    uint8_t msg_type;
    uint8_t cmd;
    uint8_t param[PARAM_BUFF];
    uint8_t param_len;
} parsed_msg_s;

typedef struct io_msg {
    uint8_t msg[IO_MSG_BUFF];
    uint8_t len;
} io_msg_s;

typedef struct braccio_angles {
    uint8_t m1;
    uint8_t m2;
    uint8_t m3;
    uint8_t m4;
    uint8_t m5;
    uint8_t m6;
} braccio_angles_s;


class braccio_arm {
    public:
        braccio_arm(Stream &serial); 
        ~braccio_arm();

        void init_arm(int soft_start_level=SOFT_START_DEFAULT); 
        void set_default_pos();

        bool serial_avail();
        void serial_read(uint8_t *buff, size_t len);
    
        int set_parsed_msg(parsed_msg_s *fill, uint8_t msg_type, uint8_t cmd,
                           uint8_t param_len, uint8_t *param);
        
        int parse_msg(uint8_t *msg, parsed_msg_s *in_msg);
        int exec_command(parsed_msg_s *in_msg);
        
        void send_ack();
        void send_finish();
        int send_print(const char *format, ...);
        int send_verbose(const char *format, ...);
        int send_error(const char *format, ...);
        
        int create_send_msg(parsed_msg_s *msg);
        int create_io_msg(parsed_msg_s *msg, io_msg_s *io_msg);
        int send_message(io_msg_s *msg);

    private:
        uint8_t check_angle(uint8_t angle, uint8_t min, uint8_t max);
        void check_all_angles(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4,
                              uint8_t a5, uint8_t a6);
        int vsnprintf_check(char *buff, int size, const char *format, 
                            va_list args); 
         
        Stream &serial;
        braccio_angles_s angles;

        /*
         * NOTE: There is a _Braccio class object declared as extern globably in
         *       Braccio.h named Braccio. We must declare our servos globally
         *       in the .cpp file for externs in Braccio.cpp. 
         *       I commented out the global braccio class extern in the llibrary
         *       header and declared my own variable here.
         *       If you downloaded Braccio.h via arduino IDE it is usually
         *       in ~/Arduino/libraries/
         *
         *       You do not need to comment it out for this to work but it may
         *       take up space.
         */

        _Braccio braccio;
};
#endif
