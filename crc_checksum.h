#ifndef CRC_CHECKSUM
#define CRC_CHECKSUM

#include <unistd.h>

unsigned short crc16(unsigned short crc, unsigned char const *buffer, size_t len);
#endif
