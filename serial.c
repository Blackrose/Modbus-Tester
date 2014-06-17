#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
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
#define BUF_SIZE 200
char buffer[BUF_SIZE];

void send_soe(int fd);
void change_vendor(int fd, int type);
void print_senddata(unsigned int len);
void set_serial_options(int, int);
unsigned short crc16(unsigned short crc, unsigned char const *buffer, size_t len);

int main(int argc, char *argv[])
{
    char *serial_dev = NULL;
    int serial_fd, read_sz;
    int i;
    unsigned int crc_checksum;
    int arg;

    if(argc < 2){
        printf("Usage:./comtool /dev/tty0\n");
        exit(-2);
    }
    
    opterr = 0;

    while((arg = getopt(argc, argv, "i:")) != -1){
        switch(arg){
            case 'i':
                serial_dev = optarg;
                break;
            default:
                abort();
        }
    }

    
   
    printf("Serial Port is %s\n", serial_dev);
    serial_fd = open(serial_dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if(serial_fd < 0){
        perror("Could not open serial prot\n");
        exit(-1);
    }
#if 0
    set_serial_options(serial_fd, EVEN_PARITY);
    send_soe(serial_fd);
    sleep(1);
    recv_data(serial_fd, buffer);
#else
    change_vendor(serial_fd, 1);
    sleep(1);
    recv_data(serial_fd, buffer);
    change_vendor(serial_fd, 2);
    sleep(1);
    recv_data(serial_fd, buffer);
#endif
    while(1){
        sleep(1);

    }
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

void send_soe(int fd)
{
    unsigned int crc_checksum;
    
    memset(buffer, 0, BUF_SIZE);
    buffer[0] = 0x01;
    buffer[1] = 0x04;
    buffer[2] = 0x41;
    buffer[3] = 0x00;
    buffer[4] = 0x00;
    buffer[5] = 0x07;
    
    crc_checksum = crc16(0xffff, buffer, 6);
    printf("crc_byte1:%x\n", crc_checksum & 0xff);
    printf("crc_byte2:%x\n", ((crc_checksum >> 8) & 0xff));
    buffer[6] = (char)(crc_checksum & 0xff);
    buffer[7] = (char)((crc_checksum >> 8) & 0xff);
    
    print_senddata(8);
    write(fd, buffer, 8);
 
}

void change_vendor(int fd, int type)
{
    unsigned int crc_checksum;
    
    memset(buffer, 0, BUF_SIZE);
    
    buffer[0] = 0x01;
    buffer[1] = 0x10;
    if(type == 1)
        buffer[2] = 0x42;
    else if(type == 2)
        buffer[2] = 0x40;
    buffer[3] = 0x78;
    buffer[4] = 0x00;
    buffer[5] = 0x01;
    buffer[6] = 0x02;
    buffer[7] = 0x00;
    buffer[8] = 0x3;
    
    crc_checksum = crc16(0xffff, buffer, 9);
    buffer[9] = (char)(crc_checksum & 0xff);
    buffer[10] = (char)((crc_checksum >> 8) & 0xff);
    
    print_senddata(11);
    write(fd, buffer, 11);
 
}

void recv_data(int fd)
{
    int read_sz, i;

    memset(buffer, 0, BUF_SIZE);
    
    read_sz = read(fd, buffer, sizeof(buffer));
    if(read_sz > 0){
        printf("Recv[%d]:", read_sz);
        for(i = 0; i < read_sz; i++)
            printf("0x%02x ", (char)buffer[i]);
        printf("\n");
    }
}

void print_senddata(unsigned int len)
{
    int i;

    printf("Sending[%d]:", len);
    for(i = 0; i < len; i++)
        printf("0x%02x ", buffer[i]);
    printf("\n");

}
