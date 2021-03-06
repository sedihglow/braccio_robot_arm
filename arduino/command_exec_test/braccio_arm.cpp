#include "braccio_arm.h"

#define SUCCESS 0
#define FAILURE -1

// these must be declared for extern variables in the Braccio library
// in Braccio.cpp 
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

braccio_arm::braccio_arm(Stream &serial): serial(serial)
{
    // set default angles into saftey position
    angles.m1 = M1_SAFE_ANGLE;
    angles.m2 = M2_SAFE_ANGLE;
    angles.m3 = M3_SAFE_ANGLE;
    angles.m4 = M4_SAFE_ANGLE;
    angles.m5 = M5_SAFE_ANGLE;
    angles.m6 = M6_SAFE_ANGLE;
}

braccio_arm::~braccio_arm()
{
}

// initialize robot arm to default position and sets variables in braccio
void braccio_arm::init_arm(int soft_start_level)
{
    send_verbose("Starting Braccio\n");
    braccio.begin(soft_start_level);
    set_default_pos();
}

void braccio_arm::set_default_pos()
{
    send_verbose("setting default position\n");

    angles.m1 = M1_SAFE_ANGLE;
    angles.m2 = M2_SAFE_ANGLE;
    angles.m3 = M3_SAFE_ANGLE;
    angles.m4 = M4_SAFE_ANGLE;
    angles.m5 = M5_SAFE_ANGLE;
    angles.m6 = M6_SAFE_ANGLE;

    braccio.ServoMovement(DFLT_STEP_DELAY, angles.m1, angles.m2, angles.m3,
                          angles.m4, angles.m5, angles.m6);

    send_verbose("default position set\n");
}

bool braccio_arm::serial_avail()
{
    if (serial.available())
        return true;
    return false;
}

int braccio_arm::serial_read(uint8_t *buff, size_t len)
{
    int num_recv;
    num_recv = serial.readBytes(buff, len); // room for '\0'
    return num_recv;
}

int braccio_arm::send_all_angles()
{
    parsed_msg_s out_msg;
    uint8_t param[NUM_ANGLES] = {angles.m1, angles.m2, angles.m3, angles.m4,
                                 angles.m5, angles.m6};

    set_parsed_msg(&out_msg, CMD_MSG, SEND_ANGLES, NUM_ANGLES, param);
    if (errno) {
        send_error("Failed to set parameters in parsed message.\n"); 
        errno = SUCCESS;
        return FAILURE;   
    }

    create_send_msg(&out_msg);
    if (errno) {
        send_error("Failed to create and send message");
        errno = SUCCESS;
        return FAILURE;
    }

    return SUCCESS;
}

void braccio_arm::send_ack()
{
    uint8_t ack[2] = {1, ACK};
    serial.write(ack, 2);
}

void braccio_arm::send_finish()
{
    uint8_t finish[2] = {1, FINISH};
    serial.write(finish, 2);
}

int braccio_arm::send_print(const char *format, ...)
{
    va_list args;
    uint8_t set_msg[PARAM_BUFF];
    parsed_msg_s out_msg;

    va_start(args, format);

    vsnprintf_check((char*)set_msg, PARAM_BUFF, format, args);
    set_parsed_msg(&out_msg, PRINT_MSG, PRINT_GENERAL, 
                   PARAM_STRLEN, set_msg);
    if (errno) {
        send_error("Failed to set parameters in parsed message.\n");
        errno = SUCCESS;
        return FAILURE;   
    }

    create_send_msg(&out_msg);
    if (errno) {
        send_error("Failed to create and send message");
        errno = SUCCESS;
        return FAILURE;
    }

    va_end(args);

    return SUCCESS;
}


int braccio_arm::send_error(const char *format, ...)
{
    va_list args;
    uint8_t set_msg[PARAM_BUFF];
    parsed_msg_s out_msg;

    va_start(args, format);

    vsnprintf_check((char*)set_msg, PARAM_BUFF, format, args);
    set_parsed_msg(&out_msg, PRINT_MSG, PRINT_ERROR, 
                   PARAM_STRLEN, set_msg);
    if (errno) {
        send_error("Failed to set parameters in parsed message.\n"); 
        errno = SUCCESS;
        return FAILURE;   
    }

    create_send_msg(&out_msg);
    if (errno) {
        send_error("Failed to create and send message\n");
        errno = SUCCESS;
        return FAILURE;
    }

    va_end(args);

    return SUCCESS;
}

int braccio_arm::send_verbose(const char *format, ...)
{
    va_list args;
    uint8_t set_msg[PARAM_BUFF];
    parsed_msg_s out_msg;

    va_start(args, format);

    vsnprintf_check((char*)set_msg, PARAM_BUFF, format, args);

    set_parsed_msg(&out_msg, PRINT_MSG, PRINT_VERBOSE, 
                   PARAM_STRLEN, set_msg);
    if (errno) {
        send_error("Failed to set parameters in parsed message.\n");
        errno = SUCCESS;
        return FAILURE;   
    }

    create_send_msg(&out_msg);
    if (errno) {
        send_error("Failed to create and send message\n");
        errno = SUCCESS;
        return FAILURE;
    }

    va_end(args);

    return SUCCESS;
}

int braccio_arm::set_parsed_msg(parsed_msg_s *fill, uint8_t msg_type, 
                                 uint8_t cmd, uint8_t param_len, 
                                 uint8_t *param)
{
    int i;
    int k;
    
    fill->msg_type = msg_type;
    fill->cmd = cmd;
    
    // If i dont do fill->param_len here compiler complains it may not be init
    fill->param_len = param_len;
    if (param_len == PARAM_STRLEN) { // treat param as string of variable length
        param_len = strnlen((char*)param, PARAM_BUFF); // excludes '\0'
        if (param_len == PARAM_BUFF && param[param_len-1] != '\n')
            return errno = EINVAL;
    } else if (param_len > PARAM_BUFF) {
        return errno = EINVAL;
    }

    fill->param_len = param_len;
    for (i=0, k=0; i < param_len; ++i, ++k)
        fill->param[i] = param[k];
        
    // exclude msg_size in size so reciever can parse 1 byte then parse the rest
    fill->msg_size = param_len + MSG_SIZE_NO_PARAM - 1; 

    return SUCCESS;
}

int braccio_arm::parse_msg(uint8_t *msg, parsed_msg_s *in_msg)
{
    uint8_t i = 0;
    uint8_t k = 0;

    // parse msg type
    in_msg->msg_type = msg[i++]; 

    // parse cmd type
    in_msg->cmd = msg[i++];
    
    // parse parameter length
    in_msg->param_len = msg[i++];

    if (in_msg->param_len > PARAM_BUFF)
        return errno = EINVAL;
    
    // parse parameters
    for (k=0; k < in_msg->param_len; ++k)
        in_msg->param[k] = msg[i++];
    
    // this is not sent by the python code but is set for initialization
    in_msg->msg_size = in_msg->param_len + MSG_SIZE_NO_PARAM - 1;

    return SUCCESS;
}

int braccio_arm::exec_command(parsed_msg_s *in_msg)
{
    if (in_msg->msg_type != CMD_MSG)
        return errno = EINVAL;

    switch (in_msg->cmd) {
    case M1_ANGLE:
        angles.m1 = check_angle(in_msg->param[0], M1_MIN_ANGLE, M1_MAX_ANGLE);
        braccio.ServoMovement(DFLT_STEP_DELAY, angles.m1, angles.m2, angles.m3,
                              angles.m4, angles.m5, angles.m6);
        send_verbose("Changed M1 angle to %u\n", angles.m1);
    break;
    case M2_ANGLE:
        angles.m2 = check_angle(in_msg->param[0], M2_MIN_ANGLE, M2_MAX_ANGLE); 
        braccio.ServoMovement(DFLT_STEP_DELAY, angles.m1, angles.m2, angles.m3,
                              angles.m4, angles.m5, angles.m6);
        send_verbose("Changed M2 angle to %u\n", angles.m2);
    break;
    case M3_ANGLE:
        angles.m3 = check_angle(in_msg->param[0], M3_MIN_ANGLE, M3_MAX_ANGLE); 
        braccio.ServoMovement(DFLT_STEP_DELAY, angles.m1, angles.m2, angles.m3,
                              angles.m4, angles.m5, angles.m6);
        send_verbose("Changed M3 angle to %u\n", angles.m3);
    break;
    case M4_ANGLE:
        angles.m4 = check_angle(in_msg->param[0], M4_MIN_ANGLE, M4_MAX_ANGLE); 
        braccio.ServoMovement(DFLT_STEP_DELAY, angles.m1, angles.m2, angles.m3,
                              angles.m4, angles.m5, angles.m6);
        send_verbose("Changed M4 angle to %u\n", angles.m4);
    break;
    case M5_ANGLE:
        angles.m5 = check_angle(in_msg->param[0], M5_MIN_ANGLE, M5_MAX_ANGLE); 
        braccio.ServoMovement(DFLT_STEP_DELAY, angles.m1, angles.m2, angles.m3,
                              angles.m4, angles.m5, angles.m6);
        send_verbose("Changed M5 angle to %u\n", angles.m5);
    break;
    case M6_ANGLE:
        angles.m6 = check_angle(in_msg->param[0], M6_MIN_ANGLE, M6_MAX_ANGLE); 
        braccio.ServoMovement(DFLT_STEP_DELAY, angles.m1, angles.m2, angles.m3,
                              angles.m4, angles.m5, angles.m6);
        send_verbose("Changed M6 angle to %u\n", angles.m6);
    break;
    case MX_ANGLE:
        check_all_angles(in_msg->param[0], in_msg->param[1], in_msg->param[2],
                         in_msg->param[3], in_msg->param[4], in_msg->param[5]);
        braccio.ServoMovement(DFLT_STEP_DELAY, angles.m1, angles.m2, angles.m3,
                              angles.m4, angles.m5, angles.m6);
        send_verbose("Changed all angles, "
                     "M1: %d, M2: %d, M3: %d, M4: %d, M5: %d, M6: %d\n", 
                     angles.m1, angles.m2, angles.m3, angles.m4, angles.m5,
                     angles.m6);
    break;
    case REQUEST_MX_ANGLE:
        send_all_angles(); 
        send_verbose("Sent all angles, "
                     "M1: %d, M2: %d, M3: %d, M4: %d, M5: %d, M6: %d\n", 
                     angles.m1, angles.m2, angles.m3, angles.m4, angles.m5,
                     angles.m6);
    break; 
    case SET_DFLT_POS:
        set_default_pos();
        break;
    default:
        send_verbose("Invalid command recieved.\n");
        return errno = EINVAL;
    }

    return SUCCESS;
}

int braccio_arm::create_send_msg(parsed_msg_s *msg)
{
    io_msg_s to_send;

    if (create_io_msg(msg, &to_send))
        return FAILURE;

    send_message(&to_send);
    if (errno)
        return FAILURE;

    return SUCCESS;
}

int braccio_arm::create_io_msg(parsed_msg_s *msg, io_msg_s *io_msg)
{
    uint8_t i = 0;
    uint8_t k = 0;

    io_msg->msg[i++] = msg->msg_size;
    io_msg->msg[i++] = msg->msg_type;
    io_msg->msg[i++] = msg->cmd;
    
    if (msg->param_len > PARAM_BUFF)
        return errno = EINVAL;

    io_msg->msg[i++] = msg->param_len;

    for (k=0; k < msg->param_len; ++k)
        io_msg->msg[i++] = msg->param[k];
    
    io_msg->len = MSG_SIZE_NO_PARAM + k;

    return SUCCESS;
}

int braccio_arm::send_message(io_msg_s *msg)
{
    int ret;

    ret = serial.write(msg->msg, msg->len);
    if (ret != msg->len) {
        errno = EIO;
        return FAILURE;
    }

    return ret;
}

                    /* private functions */

uint8_t braccio_arm::check_angle(uint8_t angle, uint8_t min, uint8_t max)
{
    if (angle < min)
        return min;

    if (angle > max)
        return max;

    return angle;
}

void braccio_arm::check_all_angles(uint8_t a1, uint8_t a2, uint8_t a3, 
                                   uint8_t a4, uint8_t a5, uint8_t a6)
{
    angles.m1 = check_angle(a1, M1_MIN_ANGLE, M1_MAX_ANGLE);
    angles.m2 = check_angle(a2, M2_MIN_ANGLE, M2_MAX_ANGLE); 
    angles.m3 = check_angle(a3, M3_MIN_ANGLE, M3_MAX_ANGLE); 
    angles.m4 = check_angle(a4, M4_MIN_ANGLE, M4_MAX_ANGLE); 
    angles.m5 = check_angle(a5, M5_MIN_ANGLE, M5_MAX_ANGLE); 
    angles.m6 = check_angle(a6, M6_MIN_ANGLE, M6_MAX_ANGLE);
}

int braccio_arm::vsnprintf_check(char *buff, int size, const char *format,
                                 va_list args)
{
    int num_written = 0;

    num_written = vsnprintf(buff, size, format, args);
    if (num_written > size-1) { // format was too large for buffer
        send_error("Future message truncated, message was to long.");
        return FAILURE;
    }

    return SUCCESS;
}
