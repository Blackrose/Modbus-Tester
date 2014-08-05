#include "modbus.h"

void read_config(int fd, unsigned int addr, unsigned length, char *buffer)
{
    unsigned int crc_checksum;
    
    
    buffer[0] = 0x01;
    buffer[1] = 0x03;
    buffer[2] = (addr >> 8) & 0xff;
    buffer[3] = addr & 0xff;
    buffer[4] = (length >> 8) & 0xff;
    buffer[5] = length & 0xff;
    
    crc_checksum = crc16(0xffff, buffer, 6);
    buffer[6] = (char)(crc_checksum & 0xff);
    buffer[7] = (char)((crc_checksum >> 8) & 0xff);
    
    print_senddata(8);
    write(fd, buffer, 8);
 
}
