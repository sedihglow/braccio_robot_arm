#include "braccio_arm.h"

#define SUCCESS 0
#define FAILURE -1

braccio_arm::braccio_arm(Stream &serial): serial(serial)
{
}

braccio_arm::~braccio_arm()
{
}

int braccio_arm::send_print(const char *format, ...)
{
    va_list args;
    uint8_t set_msg[PARAM_BUFF];
    parsed_msg_s out_msg;

    va_start(args, format);

    snprintf_check((char*)set_msg, PARAM_BUFF, format, args);
    set_parsed_msg(&out_msg, PRINT_MSG, PRINT_GENERAL, 
                   PARAM_STRING, set_msg);
    if (errno) {
        send_error("%s, Failed to set parameters in parsed message.", 
                   strerror(errno));
        errno = SUCCESS;
        return FAILURE;   
    }

    create_send_msg(&out_msg);
    if (errno) {
        send_error("%s, Failed to create and send message", strerror(errno));
        errno = 0;
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

    snprintf_check((char*)set_msg, PARAM_BUFF, format, args);
    set_parsed_msg(&out_msg, PRINT_MSG, PRINT_ERROR, 
                   PARAM_STRING, set_msg);
    if (errno) {
        send_error("%s, Failed to set parameters in parsed message.", 
                   strerror(errno));
        errno = SUCCESS;
        return FAILURE;   
    }

    create_send_msg(&out_msg);
    if (errno) {
        send_error("%s, Failed to create and send message", strerror(errno));
        errno = 0;
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

    snprintf_check((char*)set_msg, PARAM_BUFF, format, args);
    set_parsed_msg(&out_msg, PRINT_MSG, PRINT_VERBOSE, 
                   PARAM_STRING, set_msg);
    if (errno) {
        send_error("%s, Failed to set parameters in parsed message.", 
                   strerror(errno));
        errno = SUCCESS;
        return FAILURE;   
    }

    create_send_msg(&out_msg);
    if (errno) {
        send_error("%s, Failed to create and send message", strerror(errno));
        errno = 0;
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

    if (param_len == 0) { // treat param as string of variable length
        param_len = strnlen((char*)param, PARAM_BUFF-1) + 1; // for '\0'
        if (param_len == PARAM_BUFF && param[param_len-1] != '\0')
            return errno = EINVAL;
    }

    fill->param_len = param_len;
    for (i=0, k=0; i < param_len; ++i, ++k)
        fill->param[i] = param[k];

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

    for (k=0; k < in_msg->param_len; ++k)
        in_msg->param[k] = msg[i++];

    return SUCCESS;
}

/* TODO: error handling for functions called in switch */
int braccio_arm::exec_command(parsed_msg_s *in_msg)
{
    uint8_t set_msg[PARAM_BUFF] = {'\0'};
    parsed_msg_s out_msg;

    if (in_msg->msg_type != CMD_MSG)
        return errno = EINVAL;

    switch (in_msg->cmd) {
    case M1_ANGLE:
        angles.m1 = check_angle(in_msg->param[0], M1_MIN_ANGLE, M1_MAX_ANGLE);

        snprintf_check((char*)set_msg, PARAM_BUFF, 
                       "Changed M1 angle to %u\n", angles.m1);
        set_parsed_msg(&out_msg, PRINT_MSG, PRINT_VERBOSE, 
                       PARAM_STRING, set_msg);
 
        create_send_msg(&out_msg);
    break;
    case M2_ANGLE:
        angles.m2 = check_angle(in_msg->param[0], M2_MIN_ANGLE, M2_MAX_ANGLE); 
        snprintf_check((char*)set_msg, PARAM_BUFF, 
                       "Changed M2 angle to %u\n", angles.m2);
        set_parsed_msg(&out_msg, PRINT_MSG, PRINT_VERBOSE, 
                       PARAM_STRING, set_msg);

        create_send_msg(&out_msg);
    break;
    case M3_ANGLE:
        angles.m3 = check_angle(in_msg->param[0], M3_MIN_ANGLE, M3_MAX_ANGLE); 
        snprintf_check((char*)set_msg, PARAM_BUFF, 
                       "Changed M3 angle to %u\n", angles.m3);
        set_parsed_msg(&out_msg, PRINT_MSG, PRINT_VERBOSE, 
                       PARAM_STRING, set_msg);

        create_send_msg(&out_msg);
    break;
    case M4_ANGLE:
        angles.m4 = check_angle(in_msg->param[0], M4_MIN_ANGLE, M4_MAX_ANGLE); 
        snprintf_check((char*)set_msg, PARAM_BUFF, 
                       "Changed M4 angle to %u\n", angles.m4);
        set_parsed_msg(&out_msg, PRINT_MSG, PRINT_VERBOSE, 
                       PARAM_STRING, set_msg);

        create_send_msg(&out_msg);
    break;
    case M5_ANGLE:
        angles.m5 = check_angle(in_msg->param[0], M5_MIN_ANGLE, M5_MAX_ANGLE); 
        snprintf_check((char*)set_msg, PARAM_BUFF, 
                      "Changed M5 angle to %u\n", angles.m5);
        set_parsed_msg(&out_msg, PRINT_MSG, PRINT_VERBOSE, 
                       PARAM_STRING, set_msg);

        create_send_msg(&out_msg);
    break;
    case M6_ANGLE:
        angles.m6 = check_angle(in_msg->param[0], M6_MIN_ANGLE, M6_MAX_ANGLE); 
        snprintf_check((char*)set_msg, PARAM_BUFF, 
                       "Changed M6 angle to %u\n", angles.m6);
        set_parsed_msg(&out_msg, PRINT_MSG, PRINT_VERBOSE, 
                       PARAM_STRING, set_msg);

        create_send_msg(&out_msg);
    break;
    case MX_ANGLE:
        angles.m1 = check_angle(in_msg->param[0], M1_MIN_ANGLE, M1_MAX_ANGLE);
        angles.m2 = check_angle(in_msg->param[1], M2_MIN_ANGLE, M2_MAX_ANGLE); 
        angles.m3 = check_angle(in_msg->param[2], M3_MIN_ANGLE, M3_MAX_ANGLE); 
        angles.m4 = check_angle(in_msg->param[3], M4_MIN_ANGLE, M4_MAX_ANGLE); 
        angles.m5 = check_angle(in_msg->param[4], M5_MIN_ANGLE, M5_MAX_ANGLE); 
        angles.m6 = check_angle(in_msg->param[5], M6_MIN_ANGLE, M6_MAX_ANGLE);

        snprintf_check((char*)set_msg, PARAM_BUFF, "Changed all angles, "
                 "M1: %d, M2: %d, M3: %d, M4: %d, M5: %d, M6: %d\n", 
                 angles.m1, angles.m2, angles.m3, angles.m4, angles.m5,
                 angles.m6);
        set_parsed_msg(&out_msg, PRINT_MSG, PRINT_VERBOSE, 
                       PARAM_STRING, set_msg);

        create_send_msg(&out_msg);
    break;
    default:
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

uint8_t braccio_arm::check_angle(uint8_t angle, uint8_t min, uint8_t max)
{
    if (angle < min)
        return min;

    if (angle > max)
        return max;

    return angle;
}

int braccio_arm::snprintf_check(char *buff, int size, const char *format, ...)
{
    va_list args;
    int num_written = 0;

    va_start(args, format);

    num_written = vsnprintf(buff, size, format, args);
    if (num_written > size-1) { // format was too large for buffer
        send_error("Future message truncated, message was to long.");
        return FAILURE;
    }

    va_end(args);
    
    return SUCCESS;
}
