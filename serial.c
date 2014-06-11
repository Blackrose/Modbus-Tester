#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include "serial.h"

enum PARITY_TYPE{
    NO_PARITY = 'n',
    ODD_PARITY = 'o',
    EVEN_PARITY = 'e',
    SPACE_PARITY = 's',
};

#define RAW_LINE_INPUT

void set_serial_options(int, int);
unsigned short crc16(unsigned short crc, unsigned char const *buffer, size_t len);

int main(int argc, char *argv[])
{
    char serial_dev[20] = "/dev/ttyUSB0";
    int serial_fd, read_sz;
    char buffer[200];
    int i;
    unsigned int crc_checksum;

    if(argc < 2){
        printf("Usage:./comtool /dev/tty0\n");
        exit(-2);
    }
   
    printf("Serial Port is%s\n", argv[1]);
    serial_fd = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);
    if(serial_fd < 0){
        perror("Could not open serial prot\n");
        exit(-1);
    }

    set_serial_options(serial_fd, EVEN_PARITY);

    memset(buffer, 0, sizeof(buffer));

#if 1
    buffer[0] = 0x01;
    buffer[1] = 0x04;
    buffer[2] = 0x41;
    buffer[3] = 0x00;
    buffer[4] = 0x00;
    buffer[5] = 0x07;
#endif
    crc_checksum = crc16(0xffff, buffer, 6);
    printf("crc_byte1:%x\n", crc_checksum & 0xff);
    printf("crc_byte2:%x\n", ((crc_checksum >> 8) & 0xff));
    buffer[6] = (char)(crc_checksum & 0xff);
    buffer[7] = (char)((crc_checksum >> 8) & 0xff);
   
    printf("Sending data:");
    for(i = 0; i < 8; i++)
        printf("0x%02x ", buffer[i]);
    printf("\n");

    write(serial_fd, buffer, 8);
#if 1
    while(1){
    sleep(1);

    memset(buffer, 0, sizeof(buffer));
    read_sz = read(serial_fd, buffer, sizeof(buffer));
    if(read_sz > 0)
        printf("recv data count = %d\n", read_sz);
        for(i = 0; i < read_sz; i++)
            printf("0x%02x ", (char)buffer[i]);
        printf("\n");
    }
#endif
    close(serial_fd);
    return 0;
}

void set_serial_options(int fd, int parity)
{
    struct termios serial_opts;

    printf("Config the serial port\n");
    
    tcflush(fd, TCIOFLUSH);
    tcgetattr(fd, &serial_opts);

    bzero(&serial_opts, sizeof(serial_opts));

    serial_opts.c_cflag |= (B4800 | CLOCAL | CREAD);
    serial_opts.c_cflag &= ~CSIZE;
    serial_opts.c_cflag |= (CS8);
    
    // One stop bit 
    serial_opts.c_cflag &= ~CSTOPB;

    switch(parity){
        case 'n':
        case 'N':
            // no parity
            serial_opts.c_cflag &= ~PARENB;
            serial_opts.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O':
            // odd parity
            serial_opts.c_cflag |= PARENB;
            serial_opts.c_cflag |= PARODD;
            serial_opts.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E':
            // even parity
            serial_opts.c_cflag |= PARENB;
            serial_opts.c_cflag &= ~PARODD;
            serial_opts.c_iflag |= INPCK;
            break;
        case 's':
        case 'S':
            // space parity
            serial_opts.c_cflag &= ~PARENB;
            serial_opts.c_cflag &= ~CSTOPB;
            serial_opts.c_iflag |= INPCK;
            break;
        default:
            fprintf(stderr, "you must setting parity\n");
    }

    serial_opts.c_cc[VTIME] = 150;
    serial_opts.c_cc[VMIN] = 0;

#ifdef RAW_LINE_INPUT 
    // RAW input
    serial_opts.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
#else
    // Classic input
    serial_opts.c_lflag |= (ICANON | ECHO | ECHOE | ISIG);
#endif
    
    // disable software follow control
    serial_opts.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Output RAW type
    //serial_opts.c_oflag &= ~OPOST;
    
    // disable parity checking
    serial_opts.c_iflag |= INPCK;

    tcsetattr(fd, TCSANOW, &serial_opts);

    sleep(2);
}

unsigned short crc16_byte(unsigned short crc, const unsigned char data)
{
    return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}

unsigned short crc16(unsigned short crc, unsigned char const *buffer, size_t len)
{
    while(len--){
        crc = crc16_byte(crc, *buffer++);
    }

    return crc;
}
