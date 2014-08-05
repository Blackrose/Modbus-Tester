#ifndef SERIAL_H
#define SERIAL_H

#define PROG_NAME "Modbus Tester"
#define PROG_VERSION "0.0.1"

enum BOOL{
    TRUE = 1,
    FALSE = 0,
};

enum ERR_CODE{
    ERR_WRONG_ARG = 1,
    ERR_WRONG_SOCK = 2,
    ERR_WRONG_DEV = 3,
    ERR_SOCK_CONCT = 4,
    ERR_SERIAL_DEV = 5,
};

#endif
