#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
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

void read_config(int fd);
void send_soe(int fd);
void change_vendor(int fd, int type);
void print_senddata(unsigned int len);
void set_serial_options(int, int);
unsigned short crc16(unsigned short crc, unsigned char const *buffer, size_t len);

int main(int argc, char *argv[])
{
    char *serial_dev = NULL;
    int serial_fd, read_sz, socket_enable;
    int i;
    unsigned int crc_checksum;
    int arg;
    int soe_flag = 0;
    int vendor_flag = 0;
    int rconfig_flag = 0;
    struct sockaddr_in device_addr;
    int sockfd;

    if(argc < 2){
        printf("Usage:./comtool /dev/tty0\n");
        exit(-2);
    }
    
    opterr = 0;

    while((arg = getopt(argc, argv, "i:svr")) != -1){
        switch(arg){
            case 'i':
                printf("%s\n", optarg);
                if(strcmp(optarg, "socket") == 0){
                    socket_enable = 1; 
                }else
                    serial_dev = optarg;

                break;
            case 's':
                soe_flag = 1;
                break;
            case 'v':
                vendor_flag = 1;
                break;
            case 'r':
                rconfig_flag = 1;
                break;
            default:
                abort();
        }
    }

    
    if(serial_dev){
        printf("Serial Port is %s\n", serial_dev);
        serial_fd = open(serial_dev, O_RDWR | O_NOCTTY | O_NDELAY);
        if(serial_fd < 0){
            perror("Could not open serial prot\n");
            exit(-1);
        }
        set_serial_options(serial_fd, EVEN_PARITY);
    }

    if(socket_enable){
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if(sockfd < 0){
            perror("creat socket failed\n");
            exit(-1);
        }

        memset(&device_addr, 0, sizeof(device_addr));
        device_addr.sin_family = AF_INET;
        device_addr.sin_port = htons(9764);
        device_addr.sin_addr.s_addr = inet_addr("192.168.1.220");
    }

    if(soe_flag){
        if(socket_enable)
            send_soe(sockfd);
        else
            send_soe(serial_fd);
        sleep(1);
        if(socket_enable)
            recv_data(sockfd, buffer);
        else
            recv_data(serial_fd, buffer);
    }

    if(vendor_flag){
        change_vendor(serial_fd, 1);
        sleep(1);
        recv_data(serial_fd, buffer);
        change_vendor(serial_fd, 2);
        sleep(1);
        recv_data(serial_fd, buffer);
    }

    if(rconfig_flag){
        char config[100];
        unsigned int config_cnt;
        float sd_i = 0;
        int sd_t = 0;
        float config_value[30];
        int i, j, value_tmp;

        memset(config, 0, sizeof(config));
        read_config(serial_fd);
        sleep(1);
        config_cnt = recv_data(serial_fd, config);
        config_cnt = config_cnt;

        for(i = 0, j = 0; i < config_cnt; j++){
            value_tmp = (config[i] << 8) | (config[i + 1] & 0xff);
            //printf("item%d = 0x%x\n", j, value_tmp);
            config_value[j] = value_tmp / 100;

            i += 2;
        }
        //sd_i = ((config[1] << 8) | config[2]) / 100;

        printf("Config has %d bytes\n", config_cnt);
#if 0
        for(i = 0; i < config_cnt / 2; i++){
            printf("%d = %0.2f\n", i, config_value[i]);
        }
#else
        printf("SD_I = %0.2fA\n", config_value[0]);
        printf("SD_T = %0.2fS\n", config_value[1]);
        printf("XSSD_I = %0.2fA\n", config_value[2]);
        printf("XSSD_T = %0.2fS\n", config_value[3]);
        printf("GL_I = %0.2fA\n", config_value[4]);
        printf("GL_T = %0.2fS\n", config_value[5]);
        printf("Z_I = %0.2fA\n", config_value[6]);
        printf("Z_T = %0.2fS\n", config_value[7]);
        printf("FSX_I = %0.2fA\n", config_value[8]);
        printf("FSX_T = %0.2fS\n", config_value[9]);
        printf("FSX_Curve = %0.2f\n", config_value[10]);
        printf("GFH_I = %0.2fA\n", config_value[11]);
        printf("GFH_T = %0.2fS\n", config_value[12]);
        printf("SD1_I = %0.2fA\n", config_value[13]);
        printf("SD1_T = %0.2fS\n", config_value[14]);
        printf("XSSD1_I = %0.2fA\n", config_value[15]);
        printf("XSSD1_T = %0.2fS\n", config_value[16]);
        printf("GL1_I = %0.2fA\n", config_value[17]);
        printf("GL1_T = %0.2fS\n", config_value[18]);
#endif
    }

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

int recv_data(int fd, char* out)
{
    int read_sz, i;

    memset(buffer, 0, BUF_SIZE);
    
    read_sz = read(fd, buffer, sizeof(buffer));
    if(read_sz > 0){
        printf("Recv[%d]:", read_sz);
        for(i = 0; i < read_sz; i++)
            printf("0x%02x ", (buffer[i] & 0xff));
        printf("\n");
    }

    memcpy(out, buffer+3, read_sz - 2);

    return read_sz - 5;
}

void print_senddata(unsigned int len)
{
    int i;

    printf("Sending[%d]:", len);
    for(i = 0; i < len; i++)
        printf("0x%02x ", (buffer[i] & 0xff));
    printf("\n");

}

void read_config(int fd)
{
    unsigned int crc_checksum;
    
    memset(buffer, 0, BUF_SIZE);
    
    buffer[0] = 0x01;
    buffer[1] = 0x03;
    buffer[2] = 0x40;
    buffer[3] = 0xc0;
    buffer[4] = 0x00;
    buffer[5] = 0x10;
    
    crc_checksum = crc16(0xffff, buffer, 6);
    buffer[6] = (char)(crc_checksum & 0xff);
    buffer[7] = (char)((crc_checksum >> 8) & 0xff);
    
    print_senddata(8);
    write(fd, buffer, 8);
 
}

